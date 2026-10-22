"""Real decision-node callbacks with ROS transport/time replaced."""
import csv
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch
from test_merge_integration import WS, COLUMNS, geometry


class Message:
    def __init__(self, **values):self.__dict__.update(values)


class AdapterTest(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        directory=Path(self.tmp.name)
        path,circ,_=geometry()
        for name,points in (("path.csv",path),("ring.csv",circ)):
            (directory/name).write_text("\n".join("%s,%s,%s"%(p.x,p.y,p.z) for p in points))
        with (directory/"regions.csv").open("w",newline="") as stream:
            w=csv.writer(stream);w.writerow(COLUMNS)
            w.writerow(["r",10,65,20,25,40,50,55,"ring.csv",25,35])
        self.params={"~path_file":str(directory/"path.csv"),
            "~regions_file":str(directory/"regions.csv"),"~enabled":True,
            "~defaults_file":str(WS/"config/roundabout/defaults.yaml")}
        self.now=100.;self.wall=10.
        self.ros=Mock();self.ros.get_param.side_effect=lambda key,default=None:self.params.get(key,default)
        self.ros.get_time.side_effect=lambda:self.now
        self.ros.Publisher.side_effect=lambda *a,**kw:Mock()
        modules={"rospy":self.ros,"nav_msgs.msg":NS(Odometry=Message),
                 "std_msgs.msg":NS(String=Message),"morai_msgs.msg":NS(EgoVehicleStatus=Message),
                 "lidar_perception.msg":NS(LidarObstacleArray=Message)}
        file=WS/"src/mission/roundabout_mission/scripts/merge_ws_roundabout_gate.py"
        spec=importlib.util.spec_from_file_location("_roundabout_adapter",file)
        self.module=importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules,modules):spec.loader.exec_module(self.module)
        self.module.time=NS(monotonic=lambda:self.wall)
        self.node=self.module.Gate()

    def inputs(self,x=20.,speed=0.,obstacles=True,capability=True):
        header=NS(frame_id="map",stamp=NS(to_sec=lambda:self.now))
        self.node.odom(Message(header=header,pose=NS(pose=NS(position=NS(x=x,y=0.),
            orientation=NS(x=0.,y=0.,z=0.,w=1.))),twist=NS(twist=NS(linear=NS(x=speed,y=0.)))))
        self.node.vehicle_speed(Message(header=header,velocity=NS(x=speed,y=0.)))
        if obstacles:self.node.obstacles(Message(header=header,obstacle_count=0,obstacles=[]))
        if capability:self.node.path_capability(Message(data=json.dumps(dict(timestamp=self.now,
            path_sha256=self.node.path_sha,speed_unit="m/s",caps={"r":3.4}))))

    def tick(self):
        self.node.tick(None)
        return json.loads(self.node.publisher.publish.call_args.args[0].data)

    def test_adapter_publishes_status_only_and_requires_mandatory_stop(self):
        self.inputs();row=self.tick()
        self.assertTrue(row["stop_required"])
        self.assertEqual(row["decision_reason_detail"],"mandatory_entry_decision_stop")
        self.assertEqual(row["speed_unit"],"m/s")
        self.assertEqual([c.args[0] for c in self.ros.Publisher.call_args_list],["/planning/roundabout/status"])

    def test_healthy_pose_can_cruise_outside_region_without_lidar(self):
        self.inputs(x=1.,speed=2.,obstacles=False)
        row=self.tick();self.assertFalse(row["active"]);self.assertFalse(row["stop_required"])

    def test_missing_lidar_stops_inside_region(self):
        self.inputs(obstacles=False)
        self.assertTrue(self.tick()["stop_required"])

    def test_missing_vehicle_speed_is_not_assumed_stationary(self):
        self.inputs();self.node.speed._sample=None
        self.assertEqual(self.tick()["decision_reason"],"roundabout_pose_or_speed_stale")

    def test_infeasible_curve_speed_does_not_authorize_optimistic_merge(self):
        self.inputs()
        self.now+=.05;self.wall+=.05
        self.node.path_capability(Message(data=json.dumps(dict(timestamp=self.now,
            path_sha256=self.node.path_sha,speed_unit="m/s",caps={"r":1.0}))))
        row=self.tick()
        self.assertTrue(row["stop_required"])
        self.assertEqual(row["decision_reason"],"roundabout_path_capability_stale_or_too_low")

    def test_fresh_lidar_does_not_hide_dead_path_capability(self):
        self.inputs();self.now+=.31;self.wall+=.31
        self.inputs(capability=False)
        self.assertTrue(self.tick()["stop_required"])

    def test_clock_reset_discards_entry_permission_and_waits_for_new_inputs(self):
        self.inputs();self.tick()
        self.node.engines[0][1].state="COMMIT"
        self.node.engines[0][1].entry_was_authorized=True
        self.now=1.;self.wall+=.05
        self.assertTrue(self.tick()["stop_required"])
        self.assertFalse(self.node.engines[0][1].entry_was_authorized)
        self.inputs();self.assertTrue(self.tick()["stop_required"])


if __name__=="__main__":unittest.main()
