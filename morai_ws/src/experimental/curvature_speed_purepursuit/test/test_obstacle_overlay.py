"""Actual PP callbacks with avoidance and roundabout decisions composed."""
import json
import unittest
from types import SimpleNamespace as NS
import test_node_startup as fixtures
from purepursuit_mgeo.avoidance_gate import AvoidanceGate
from purepursuit_mgeo.mission_speed_gate import MissionSpeedGate


class ObstacleOverlayTests(unittest.TestCase):
    def setUp(self):
        self.f=fixtures.StartupTest()
        self.f.setUp()
        self.f.params["~command_topic"]="/control/ctrl_cmd"
        self.f.node=self.f.module.CurvatureSpeedPurePursuitNode()
        self.n=self.f.node
        self.n.avoidance_gate=AvoidanceGate(True,.8,"route")
        self.n.avoidance_vehicle_pub=fixtures.Mock()

    def frame(self,active=True,stop=False,offset=.002):
        self.f.now+=.05
        row=dict(timestamp=self.f.now,path_sha256="route",frame_id="map",speed_unit="m/s",
                 configured=True,active=active,stop_required=stop,emergency=False,target_speed=6.,
                 points=[[i*.2,offset*(i*.2)**2,0.] for i in range(121)] if active else [])
        self.n.avoidance_callback(NS(data=json.dumps(row)))
        return self.f.tick()

    def test_local_path_steers_and_speed_is_capped_before_existing_pi(self):
        for _ in range(100): command=self.frame()
        self.assertGreater(command.steering,0.)
        self.assertAlmostEqual(self.n.command_speed_mps,6.)
        self.assertAlmostEqual(self.n.speed_command_pub.publish.call_args.args[0].data,21.6)

    def test_signal_input_command_remains_nominal_topic(self):
        self.assertEqual(self.n.command_topic,"/control/ctrl_cmd")

    def test_lane_centering_cannot_pull_vehicle_away_from_avoidance_path(self):
        self.n.lane_centered_steering=fixtures.Mock(side_effect=AssertionError("lane assist used"))
        self.frame()
        self.n.lane_centered_steering.assert_not_called()

    def test_fresh_normal_status_does_not_globally_limit_speed_to_avoidance_speed(self):
        for _ in range(100): self.frame(active=False)
        self.assertGreater(self.n.command_speed_mps,6.)

    def test_missing_planner_message_causes_brake(self):
        command=self.f.tick()
        self.assertEqual((command.accel,command.brake),(0.,1.))
        self.assertFalse(self.n.goal_pub.publish.call_args.args[0].data)

    def test_old_decision_expires_and_brakes(self):
        self.frame()
        self.f.now+=.81
        command=self.f.tick()
        self.assertEqual((command.accel,command.brake),(0.,1.))

    def test_roundabout_yield_dominates_avoidance_permission(self):
        self.n.roundabout_gate=MissionSpeedGate(True,.3,"route")
        self.n.roundabout_gate.observe(dict(timestamp=self.f.now,path_sha256="route",speed_unit="m/s",
            configured=True,active=True,stop_required=True,emergency=False,target_speed=0.),self.f.now,self.f.now)
        command=self.frame()
        self.assertEqual((command.accel,command.brake),(0.,1.))

    def test_roundabout_lower_cap_is_preserved(self):
        self.n.roundabout_gate=MissionSpeedGate(True,.3,"route")
        for _ in range(100):
            self.n.roundabout_gate.observe(dict(timestamp=self.f.now,path_sha256="route",speed_unit="m/s",
                configured=True,active=True,stop_required=False,emergency=False,target_speed=2.8),self.f.now,self.f.now)
            self.frame()
        self.assertAlmostEqual(self.n.command_speed_mps,2.8)

    def test_capability_vehicle_input_publishes_while_waiting_without_deadlock(self):
        self.f.tick()
        row=json.loads(self.n.avoidance_vehicle_pub.publish.call_args.args[0].data)
        self.assertEqual(row["speed_unit"],"m/s")
        self.assertEqual(row["path_sha256"],"route")
        self.assertEqual(row["vehicle"],[0.,0.,0.,0.])


if __name__=="__main__":unittest.main()
