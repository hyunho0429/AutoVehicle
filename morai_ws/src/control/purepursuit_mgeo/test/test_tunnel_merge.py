"""Planner policy and decision contract; real geometry, no ROS/UDP."""
import json
import math
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET
import yaml
from purepursuit_mgeo.path import PathPoint
from purepursuit_mgeo.obstacle_avoidance import AvoidanceEngine
from purepursuit_mgeo.avoidance_gate import AvoidanceGate
from purepursuit_mgeo.tunnel_frenet_planner import TrackedObstacle

WS = Path(__file__).resolve().parents[4]


def box(x=20., y=0., length=2., width=1., track=1):
    return TrackedObstacle(track, x, y, length, width, 0., 0., 0., "STATIC", 0., True, "BBOX")


class EngineTests(unittest.TestCase):
    def engine(self, **changes):
        config = yaml.safe_load((WS/"config/avoidance/defaults.yaml").read_text())
        config.update(changes)
        return AvoidanceEngine([PathPoint(i,0.,0.) for i in range(201)], config, loop=False)

    def test_clear_road_preserves_normal_speed_and_no_local_path(self):
        e=self.engine()
        r=e.step((0.,0.,0.,6.),0.,[],100.,100.)
        self.assertFalse(r["active"])
        self.assertFalse(r["stop_required"])
        self.assertEqual(r["points"],[])

    def test_blocker_selects_checked_nonzero_local_path(self):
        e=self.engine()
        r=e.step((0.,0.,0.,6.),0.,[box()],100.,100.)
        self.assertTrue(r["active"],r)
        self.assertGreater(abs(r["candidate_offset_m"]),1.)
        self.assertEqual(r["target_speed"],6.)
        self.assertGreater(r["candidate_clearance_m"],0.)

    def test_original_reference_fallback_is_explicit_default(self):
        e=self.engine()
        r=e.step((0.,0.,0.,6.),0.,[box(x=1.5)],100.,100.)
        self.assertFalse(r["active"])
        self.assertTrue(r["reference_fallback"])
        self.assertFalse(r["stop_required"])

    def test_optional_stop_policy_is_configurable_without_rewriting_planner(self):
        e=self.engine(failure_policy="stop")
        r=e.step((0.,0.,0.,6.),0.,[box(x=1.5)],100.,100.)
        self.assertTrue(r["stop_required"])
        self.assertFalse(r["reference_fallback"])

    def test_normal_and_tunnel_costs_restore_without_changing_candidates(self):
        e=self.engine()
        e.step((0.,0.,0.,6.),0.,[],100.,100.,(100.,0.,0.))
        self.assertEqual((e.planner.weight_offset,e.planner.weight_safety),(100.,4.))
        offsets=list(e.offsets);times=list(e.times)
        e.step((0.,0.,0.,6.),0.,[],101.1,101.1)
        self.assertTrue(e.tunnel)
        self.assertEqual((e.planner.weight_offset,e.planner.weight_safety,e.planner.weight_clearance),(5.,40.,100.))
        for t in (102.,102.5,103.01):
            e.step((0.,0.,0.,6.),0.,[],t,t,(t,0.,0.))
        self.assertFalse(e.tunnel)
        self.assertEqual((e.planner.weight_offset,e.planner.weight_safety),(100.,4.))
        self.assertEqual((e.offsets,e.times),(offsets,times))

    def test_valid_path_is_retained_instead_of_restarting_lateral_motion(self):
        e=self.engine()
        e.step((0.,0.,0.,6.),0.,[box()],100.,100.)
        candidate=e.candidate
        e.step((.1,0.,0.,6.),.1,[box()],100.2,100.2)
        self.assertIs(e.candidate,candidate)

    def test_both_tunnel_walls_are_checked_for_every_candidate(self):
        e=self.engine()
        r=e.step((0.,0.,0.,6.),0.,[box(),box(y=4.5,length=60.,width=.2,track=2),
                                             box(y=-4.5,length=60.,width=.2,track=3)],100.,100.)
        self.assertTrue(r["active"])
        self.assertLess(abs(r["candidate_offset_m"]),4.)

    def test_local_reference_does_not_pick_opposing_crossing_branch(self):
        e=self.engine()
        # Sample only the branch selected by the existing driver's progress.
        e.route = type(e.route)([PathPoint(0.,0.,0.),PathPoint(100.,0.,0.),
                                PathPoint(100.,2.,0.),PathPoint(0.,2.,0.)])
        ref=e._reference(152.,6.)
        self.assertTrue(all(p.y==2. for p in ref.points))

    def test_loop_seam_has_a_forward_planning_horizon(self):
        points=[PathPoint(50*math.cos(i*math.pi/100),50*math.sin(i*math.pi/100),0.) for i in range(201)]
        config=yaml.safe_load((WS/"config/avoidance/defaults.yaml").read_text())
        e=AvoidanceEngine(points,config,loop=True)
        ref=e._reference(e.route.length-1.,6.)
        self.assertGreater(ref.length,55.)
        self.assertTrue(all(math.hypot(b.x-a.x,b.y-a.y)<1. for a,b in zip(ref.points,ref.points[1:])))


class ContractTests(unittest.TestCase):
    def setUp(self):
        self.g=AvoidanceGate(True,.8,"route")

    def row(self,**kw):
        row=dict(timestamp=100.,path_sha256="route",frame_id="map",speed_unit="m/s",
            configured=True,active=True,stop_required=False,emergency=False,target_speed=6.,
            points=[[i*.2,.01*(i*.2)**2,0.] for i in range(51)])
        row.update(kw)
        return row

    def select(self,speed=12.,x=0.,now=100.,wall=10.):
        return self.g.select(speed,now,wall,(x,0.,0.,0.),3.,1.)

    def test_atomic_path_and_speed_and_lower_existing_cap(self):
        self.assertTrue(self.g.observe(self.row(),100.,10.))
        cap,stop,steer,_=self.select(speed=2.8)
        self.assertEqual(cap,2.8)
        self.assertFalse(stop)
        self.assertGreater(steer[0],0.)

    def test_source_dropout_never_keeps_an_old_avoidance_path_alive(self):
        self.g.observe(self.row(),100.,10.)
        self.assertTrue(self.select(now=100.81,wall=10.81)[1])

    def test_ros_pause_expires_on_wall_clock(self):
        self.g.observe(self.row(),100.,10.)
        self.assertTrue(self.select(now=100.,wall=10.81)[1])

    def test_replay_and_conflicting_same_stamp_cannot_authorize_motion(self):
        self.g.observe(self.row(),100.,10.)
        self.assertFalse(self.g.observe(self.row(),100.2,10.2))
        self.assertFalse(self.g.observe(self.row(stop_required=True),100.2,10.2))
        self.assertTrue(self.select()[1])

    def test_malformed_path_wrong_frame_units_and_route_stop(self):
        for kw in (dict(frame_id="base_link"),dict(speed_unit="km/h"),dict(path_sha256="other"),
                   dict(points=[[0.,0.,0.],[10.,0.,0.],[20.,0.,0.]]),
                   dict(points=[[0.,0.,0.]]),dict(target_speed=float("nan")),
                   dict(points=[[0.,0.,0.],[.2,float("nan"),0.],[.4,0.,0.]])):
            self.setUp()
            self.assertFalse(self.g.observe(self.row(**kw),100.,10.))
            self.assertTrue(self.select()[1])

    def test_teleported_pose_cannot_join_old_path(self):
        self.g.observe(self.row(),100.,10.)
        self.assertTrue(self.select(x=30.)[1])

    def test_reference_fallback_is_normal_cruise_not_an_active_speed_cap(self):
        self.g.observe(self.row(active=False,points=[],target_speed=0.,reference_fallback=True),100.,10.)
        self.assertEqual(self.select()[:3],(12.,False,None))

    def test_stop_policy_brakes_even_without_local_path(self):
        self.g.observe(self.row(active=False,points=[],target_speed=0.,stop_required=True),100.,10.)
        self.assertTrue(self.select()[1])

    def test_disabled_feature_preserves_existing_nominal_control(self):
        self.g=AvoidanceGate(False)
        self.assertEqual(self.select()[:3],(12.,False,None))


class LaunchTests(unittest.TestCase):
    def test_single_shared_lidar_and_single_final_command_chain(self):
        root=ET.parse(WS/"src/bringup/morai_bringup/launch/merge_ws_drive.launch").getroot()
        includes=root.findall(".//include[@file='$(find lidar_perception)/launch/lidar_algorithm_pipeline.launch']")
        self.assertEqual(len(includes),1)
        self.assertIn("obstacle_avoidance_enabled",includes[0].get("if"))
        self.assertIn("roundabout_enabled",includes[0].get("if"))
        self.assertEqual(root.find("arg[@name='avoidance_failure_policy']").get("default"),"reference")
        pp=root.find("node[@type='curvature_speed_purepursuit_node.py']")
        self.assertEqual(pp.find("param[@name='obstacle_avoidance_enabled']").get("value"),"$(arg obstacle_avoidance_enabled)")
        script=(WS/"src/control/purepursuit_mgeo/scripts/merge_ws_obstacle_planner.py").read_text()
        self.assertNotIn("CtrlCmd",script.split('"""')[-1])


if __name__=="__main__": unittest.main()
