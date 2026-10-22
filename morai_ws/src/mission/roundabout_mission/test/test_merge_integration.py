"""No ROS/UDP: explicit regions, V6.1 decisions, protocol and pedal integration."""
import csv
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
import yaml

WS = Path(__file__).resolve().parents[4]
for relative in ("src/mission/roundabout_mission/src", "src/control/purepursuit_mgeo/src"):
    sys.path.insert(0, str(WS / relative))
from purepursuit_mgeo.path import PathPoint, load_mgeo_path
from purepursuit_mgeo.longitudinal_controller import SpeedPIController
from purepursuit_mgeo.mission_speed_gate import MissionSpeedGate
from roundabout_mission.core import Ego, Track, Decision
from roundabout_mission.regions import COLUMNS, load_regions


def geometry():
    path = [PathPoint(float(x), 0., 0.) for x in range(101)]
    circ = [PathPoint(45., float(y), 0.) for y in range(-30, 31)]
    cfg = yaml.safe_load((WS / "config/roundabout/defaults.yaml").read_text())
    cfg.update(mission_indices=dict(approach=10, stop_line=20, entry=25, conflict_start=40,
                                   conflict_end=50, merge_complete=55, exit=65),
               circulating_conflict_indices=[25, 35], roi_map=[40, 50, -35, 35],
               entry_decision_stop_index=20, ego_front_m=3.845, max_accel=1.0,
               comfortable_decel=1.5)
    return path, circ, cfg


class RegionTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.path, self.circ, self.cfg = geometry()
        self.csv = self.root / "regions.csv"
        (self.root / "ring.csv").write_text("\n".join("%s,%s,%s" % (p.x, p.y, p.z) for p in self.circ))

    def save(self, rows):
        with self.csv.open("w", newline="") as stream:
            writer = csv.writer(stream); writer.writerow(COLUMNS); writer.writerows(rows)

    def test_repository_has_no_invented_roundabout_region(self):
        self.assertEqual(load_regions(str(WS / "config/roundabout/regions.csv"),
                                     self.path, self.cfg, load_mgeo_path), [])

    def test_empty_csv_does_not_classify_a_curved_route(self):
        self.save([])
        curve = [PathPoint(math.cos(i/10.), math.sin(i/10.), 0) for i in range(100)]
        self.assertEqual(load_regions(str(self.csv), curve, self.cfg, load_mgeo_path), [])

    def test_start_and_end_alone_never_guess_stop_or_conflict_geometry(self):
        self.save([["r", 10, 65] + [""] * 8])
        with self.assertRaisesRegex(ValueError, "unfilled"):
            load_regions(str(self.csv), self.path, self.cfg, load_mgeo_path)

    def test_explicit_short_route_is_valid_without_original_1792_index(self):
        self.save([["r",10,65,20,25,40,50,55,"ring.csv",25,35]])
        name, engine = load_regions(str(self.csv), self.path, self.cfg, load_mgeo_path)[0]
        self.assertEqual((name, engine.entry_decision_stop_s), ("r",20.))
        engine.state="FOLLOW";engine.entry_stop_completed=True
        engine.update_tracks([],100.)
        self.assertFalse(engine.step(Ego(56.,0.,0.,2.),100.)["stop_required"])

    def test_overlapping_regions_are_rejected(self):
        row=["r",10,65,20,25,40,50,55,"ring.csv",25,35]
        self.save([row,["second"]+row[1:]])
        with self.assertRaisesRegex(ValueError, "overlap"):
            load_regions(str(self.csv), self.path, self.cfg, load_mgeo_path)


class DecisionTest(unittest.TestCase):
    def setUp(self):
        self.path, self.circ, self.cfg = geometry()
        self.engine = Decision(self.path, self.circ, self.cfg)

    def test_requires_stationary_stop_before_empty_gap_can_commit(self):
        e = self.engine
        e.update_tracks([],100.)
        first=e.step(Ego(20.,0.,0.,1.),100.)
        self.assertTrue(first["stop_required"])
        self.assertFalse(e.entry_stop_completed)
        e.update_tracks([],100.05); e.step(Ego(20.,0.,0.,0.),100.05)
        for i in range(1,16):
            now=100.05+i*.05;e.update_tracks([],now)
            row=e.step(Ego(20.,0.,0.,0.),now)
        self.assertEqual(row["state"],"COMMIT")
        self.assertTrue(row["allowed"])

    def test_crossing_vehicle_with_changing_id_blocks_entry(self):
        e=self.engine
        for i in range(15):
            now=100.+i*.05
            e.update_tracks([Track(i,45.,-26.+i*.15,math.pi/2,4.5,1.8,0.,3.)],now)
            row=e.step(Ego(20.,0.,0.,0.),now)
        self.assertFalse(row["allowed"])
        self.assertTrue(row["stop_required"])

    def test_stale_obstacles_stop_only_when_region_active(self):
        self.assertFalse(self.engine.step(Ego(1.,0.,0.,2.),100.,True,False)["stop_required"])
        self.assertTrue(self.engine.step(Ego(20.,0.,0.,0.),100.05,True,False)["stop_required"])

    def test_current_rear_overlap_is_not_a_following_exception(self):
        e=self.engine;e.state="COMMIT";e.entry_stop_completed=True
        e.update_tracks([Track(1,39.,0.,0.,4.5,1.8,2.,0.)],100.)
        row=e.step(Ego(40.,0.,0.,2.),100.)
        self.assertTrue(row["emergency"])


class ProtocolTest(unittest.TestCase):
    def setUp(self):
        self.gate=MissionSpeedGate(True,.3,"route")

    def row(self, **kw):
        r=dict(timestamp=100.,path_sha256="route",speed_unit="m/s",configured=True,
               active=True,stop_required=False,emergency=False,target_speed=2.8,allowed=True)
        r.update(kw);return r

    def test_speed_cap_is_in_metres_per_second_before_real_PI(self):
        self.gate.observe(self.row(),100.,10.)
        target,stop,_=self.gate.constrain(35/3.6,100.,10.)
        self.assertEqual(target,2.8)
        pedal=SpeedPIController().update(target*3.6,35.,.05,stop=stop)
        self.assertGreater(pedal.brake,0.)
        self.assertEqual(pedal.accel,0.)

    def test_false_allowed_outside_region_is_normal_cruise(self):
        self.gate.observe(self.row(active=False,allowed=False),100.,10.)
        self.assertEqual(self.gate.constrain(10.,100.,10.)[:2],(10.,False))

    def test_wait_or_emergency_dominates_nominal_acceleration(self):
        for flag in ("stop_required","emergency"):
            self.gate.observe(self.row(**{flag:True}),100.,10.)
            target,stop,_=self.gate.constrain(10.,100.,10.)
            command=SpeedPIController().update(target*3.6,0.,.05,stop=stop)
            self.assertEqual((command.accel,command.brake),(0.,1.))
            self.setUp()

    def test_missing_and_replayed_status_cannot_authorize_motion(self):
        self.assertTrue(self.gate.constrain(10.,100.,10.)[1])
        self.gate.observe(self.row(),100.,10.)
        self.assertFalse(self.gate.observe(self.row(),100.2,10.2))
        self.assertTrue(self.gate.constrain(10.,100.31,10.31)[1])

    def test_paused_ros_clock_still_expires_on_wall_clock(self):
        self.gate.observe(self.row(),100.,10.)
        self.assertTrue(self.gate.constrain(10.,100.,10.31)[1])

    def test_conflicting_same_stamp_status_revokes_permission(self):
        self.gate.observe(self.row(),100.,10.)
        self.assertFalse(self.gate.observe(self.row(stop_required=True),100.,10.))
        self.assertTrue(self.gate.constrain(10.,100.,10.)[1])

    def test_wrong_route_units_and_malformed_flags_stop(self):
        for kw in (dict(path_sha256="other"),dict(speed_unit="km/h"),dict(active="false"),
                   dict(target_speed=float("nan")),dict(configured=False)):
            self.gate.observe(self.row(**kw),100.,10.)
            self.assertTrue(self.gate.constrain(10.,100.,10.)[1])

    def test_disabled_gate_preserves_existing_driving(self):
        gate=MissionSpeedGate(False)
        self.assertEqual(gate.constrain(10.,100.,10.)[:2],(10.,False))

    def test_active_gate_never_raises_lower_curvature_speed(self):
        self.gate.observe(self.row(),100.,10.)
        self.assertEqual(self.gate.constrain(1.5,100.,10.)[0],1.5)


class LaunchTest(unittest.TestCase):
    def test_single_pedal_chain_and_default_roundabout_disabled(self):
        root=ET.parse(WS/"src/bringup/morai_bringup/launch/merge_ws_drive.launch").getroot()
        self.assertEqual(root.find("arg[@name='roundabout_enabled']").get("default"),"false")
        pp=root.find("node[@type='curvature_speed_purepursuit_node.py']")
        fusion=root.find("node[@type='maneuver_fusion_node.py']")
        self.assertEqual(pp.find("param[@name='command_topic']").get("value"),"/control/ctrl_cmd")
        self.assertEqual(fusion.find("param[@name='output_command_topic']").get("value"),"/ctrl_cmd")
        self.assertIsNone(root.find(".//node[@type='roundabout_test_driver.py']"))
        self.assertIsNotNone(root.find(".//node[@type='merge_ws_roundabout_gate.py']"))

    def test_launch_arg_defaults_only_reference_preceding_args(self):
        import re
        root=ET.parse(WS/"src/bringup/morai_bringup/launch/merge_ws_drive.launch").getroot()
        defined=set()
        for arg in root.findall("arg"):
            for needed in re.findall(r"\$\(arg ([^)]+)\)",arg.get("default","")):
                self.assertIn(needed,defined)
            defined.add(arg.get("name"))

    def test_installer_lists_new_imports_and_tracking_runtime(self):
        import os, shutil, subprocess
        bash=shutil.which("bash")
        if os.name=="nt" and Path("C:/Program Files/Git/bin/bash.exe").exists():
            bash="C:/Program Files/Git/bin/bash.exe"
        if not bash:self.skipTest("bash unavailable")
        output=subprocess.check_output([bash,"-c",'source "$1"; curvature_runtime_files "$2"',
            "manifest",str(WS/"docker/final_ws/curvature_runtime_files.sh"),str(WS)],text=True)
        paths=set(output.splitlines())
        for relative in ("src/control/purepursuit_mgeo/src/purepursuit_mgeo/mission_speed_gate.py",
                         "src/mission/roundabout_mission/scripts/merge_ws_roundabout_gate.py",
                         "src/detection/lidar_perception/scripts/lidar_tracking_node.py",
                         "config/roundabout/regions.csv",
                         "src/bringup/morai_bringup/launch/merge_ws_drive.launch"):
            self.assertIn(relative,paths)
        for relative in paths:self.assertTrue((WS/relative).is_file(),relative)
        self.assertNotIn("docker/merge_ws/highway.env",paths)
        self.assertNotIn("docker/merge_ws/highway-test.env",paths)


if __name__ == "__main__":unittest.main()
