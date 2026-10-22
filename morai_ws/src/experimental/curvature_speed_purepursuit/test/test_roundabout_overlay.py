"""Actual PP callbacks: mission targets compose before the existing pedal PI."""
import unittest
import test_node_startup as fixtures
from purepursuit_mgeo.mission_speed_gate import MissionSpeedGate


class RoundaboutOverlayTest(unittest.TestCase):
    def setUp(self):
        self.case=fixtures.StartupTest()
        self.case.setUp()
        self.node=self.case.node
        self.node.roundabout_gate=MissionSpeedGate(True,.3,"route")

    def frame(self, active=True, stop=False, speed=0.):
        t=self.case.now
        self.node.roundabout_gate.observe(dict(timestamp=t,path_sha256="route",speed_unit="m/s",
            configured=True,active=active,stop_required=stop,emergency=False,
            allowed=active and not stop,target_speed=2.8),t,t)
        return self.case.tick(speed=speed)

    def test_cap_reaches_real_pi_without_becoming_a_kph_value(self):
        for _ in range(100):self.frame()
        self.assertAlmostEqual(self.node.command_speed_mps,2.8)
        self.assertAlmostEqual(self.node.speed_command_pub.publish.call_args.args[0].data,10.08)

    def test_yield_stop_brakes_and_is_not_reported_as_route_goal(self):
        for _ in range(20):self.frame()
        command=self.frame(stop=True)
        self.assertEqual((command.accel,command.brake),(0.,1.))
        self.assertFalse(self.node.goal_pub.publish.call_args.args[0].data)

    def test_dead_decision_stream_cannot_keep_accelerating(self):
        for _ in range(20):self.frame()
        self.case.now+=.31
        command=self.case.tick()
        self.assertEqual((command.accel,command.brake),(0.,1.))

    def test_outside_region_allows_normal_curve_planning(self):
        for _ in range(100):command=self.frame(active=False)
        self.assertGreater(self.node.command_speed_mps,2.8)
        self.assertGreater(command.accel,0.)

    def test_planner_publishes_feasible_region_cap_while_waiting_for_gate(self):
        import tempfile,json
        from pathlib import Path
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/"regions.csv"
            p.write_text("region_id,start_index,end_index\nr,10,65\n")
            self.case.params.update({"~roundabout_gate_enabled":True,"~roundabout_regions_file":str(p)})
            self.case.module.route_digest=lambda _:"route"
            self.case.node=self.case.module.CurvatureSpeedPurePursuitNode()
            self.node=self.case.node
            command=self.case.tick()
            row=json.loads(self.node.roundabout_capability_pub.publish.call_args.args[0].data)
            self.assertEqual(row["path_sha256"],"route")
            self.assertEqual(row["speed_unit"],"m/s")
            self.assertGreater(row["caps"]["r"],0.)
            self.assertEqual(command.brake,1.)  # Capability publishes before gate permission.


if __name__ == "__main__":unittest.main()
