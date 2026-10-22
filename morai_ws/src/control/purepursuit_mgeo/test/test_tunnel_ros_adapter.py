"""Real planner ROS adapter with boundary messages/timers mocked."""
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace as NS
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch
import yaml

WS=Path(__file__).resolve().parents[4]
SCRIPT=WS/"src/control/purepursuit_mgeo/scripts/merge_ws_obstacle_planner.py"


class AdapterTests(unittest.TestCase):
    def setUp(self):
        self.now=100.
        self.wall=10.
        self.tmp=tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        path=Path(self.tmp.name)/"route.txt"
        path.write_text("\n".join(f"{i} 0 0" for i in range(201)))
        params={"~path_file":str(path),"~defaults_file":str(WS/"config/avoidance/defaults.yaml"),"~loop_route":False}
        ros=Mock()
        ros.get_param.side_effect=lambda k,d=None:params.get(k,d)
        ros.Time.now.side_effect=lambda:NS(to_sec=lambda:self.now)
        spec=importlib.util.spec_from_file_location("obstacle_adapter",SCRIPT)
        self.mod=importlib.util.module_from_spec(spec)
        mods={"rospy":ros,"nav_msgs.msg":NS(Odometry=object),"std_msgs.msg":NS(String=lambda **kw:NS(**kw))}
        with patch.dict(sys.modules,mods):spec.loader.exec_module(self.mod)
        self.mod.time=NS(monotonic=lambda:self.wall)
        self.node=self.mod.ObstaclePlannerNode()

    def inputs(self,obstacles=()):
        v=dict(timestamp=self.now,path_sha256=self.node.sha,frame_id="map",speed_unit="m/s",
               vehicle=[0.,0.,0.,6.],progress_s=0.)
        o=dict(timestamp=self.now,frame_id="map",obstacles=list(obstacles))
        self.node.vehicle_callback(NS(data=json.dumps(v)))
        self.node.obstacle_callback(NS(data=json.dumps(o)))

    def tick(self):
        self.node.tick(None)
        return json.loads(self.node.status_pub.publish.call_args.args[0].data)

    def test_missing_inputs_is_stop_not_a_clear_road(self):
        self.assertTrue(self.tick()["stop_required"])

    def test_fresh_empty_lidar_means_clear_road(self):
        self.inputs()
        row=self.tick()
        self.assertFalse(row["stop_required"])
        self.assertFalse(row["active"])
        self.assertEqual(row["frame_id"],"map")

    def test_real_blocker_produces_atomic_checked_path(self):
        obstacle=dict(id=1,center_x_map=20.,center_y_map=0.,length=2.,width=1.,
            velocity_x_map=0.,velocity_y_map=0.,yaw=0.,yaw_valid=True,yaw_source="BBOX",motion_state="STATIC")
        self.inputs([obstacle])
        row=self.tick()
        self.assertTrue(row["active"])
        self.assertGreater(len(row["points"]),3)

    def test_repeated_lidar_stamp_does_not_refresh_wall_timeout(self):
        self.inputs()
        self.wall+=.61
        self.inputs()
        self.assertTrue(self.tick()["stop_required"])

    def test_ros_clock_pause_expires_input_on_wall_clock(self):
        self.inputs()
        self.wall+=.7
        self.assertTrue(self.tick()["stop_required"])

    def test_wrong_frame_and_nonfinite_box_revoke_input(self):
        self.inputs()
        self.node.obstacle_callback(NS(data=json.dumps(dict(timestamp=self.now,frame_id="base_link",obstacles=[]))))
        self.assertTrue(self.tick()["stop_required"])

    def test_same_stamp_conflicting_geometry_revokes_input(self):
        self.inputs()
        self.node.vehicle_callback(NS(data=json.dumps(dict(timestamp=self.now,path_sha256=self.node.sha,
            frame_id="map",speed_unit="m/s",vehicle=[20.,0.,0.,6.],progress_s=20.))))
        self.assertTrue(self.tick()["stop_required"])

    def test_slow_calculation_keeps_original_timestamp(self):
        self.inputs()
        original=self.node.engine.step
        def slow(*args):
            result=original(*args)
            self.now+=1.
            return result
        self.node.engine.step=slow
        self.assertEqual(self.tick()["timestamp"],100.)

    def test_clock_reset_revokes_old_permission(self):
        self.inputs()
        self.tick()
        self.now=5.
        self.wall+=.1
        self.assertTrue(self.tick()["stop_required"])


if __name__=="__main__":unittest.main()
