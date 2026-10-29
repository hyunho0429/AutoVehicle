import math
import unittest
from types import SimpleNamespace as P
from purepursuit_mgeo.dynamic_zone_guard import DynamicZoneGuard


class SpeedThresholdTests(unittest.TestCase):
    def setUp(self):
        self.points = [P(x=float(i), y=0., z=0.) for i in range(201)]
        self.regions = [dict(name='dynamic_obstacle', start_xyz=[40,0,0], end_xyz=[80,0,0])]
        self.g = DynamicZoneGuard(self.points, self.regions, approach_m=10.)
        self.t = 100.

    def obstacle(self, vx=2., vy=0., y=0., key=1):
        return dict(id=key, center_x_map=50., center_y_map=y, length=1., width=1.,
                    velocity_x_map=vx, velocity_y_map=vy, yaw=0., yaw_valid=True,
                    motion_state='MOVING')

    def tick(self, obs, x=45.):
        self.t += .2
        sample = (dict(timestamp=self.t, obstacles=obs), self.t)
        return self.g.step(self.t, self.t, (x,0.), 0., sample)

    def test_below_threshold_does_not_start_stop(self):
        for v in (0., .5, 1.99, 1.999999):
            r = self.tick([self.obstacle(vx=v)])
            self.assertFalse(r['stop'])
            self.assertEqual(r['eligible_obstacle_ids'], [])
            self.assertAlmostEqual(r['cap_mps'], 20/3.6)

    def test_exact_threshold_stops(self):
        self.assertTrue(self.tick([self.obstacle()])['stop'])

    def test_above_threshold_stops(self):
        self.assertTrue(self.tick([self.obstacle(vx=3.)])['stop'])

    def test_reverse_direction_uses_magnitude(self):
        self.assertTrue(self.tick([self.obstacle(vx=-2.)])['stop'])

    def test_both_map_components_used(self):
        self.assertTrue(self.tick([self.obstacle(vx=1.2, vy=1.6)])['stop'])

    def test_lateral_speed_qualifies(self):
        self.assertTrue(self.tick([self.obstacle(vx=0., vy=2.)])['stop'])

    def test_fast_adjacent_obstacle_does_not_stop(self):
        self.assertFalse(self.tick([self.obstacle(y=5.)])['stop'])

    def test_slow_predicted_crossing_ignored(self):
        self.assertFalse(self.tick([self.obstacle(vx=0., vy=-1.9, y=4.)])['stop'])

    def test_fast_predicted_crossing_stops(self):
        self.assertTrue(self.tick([self.obstacle(vx=0., vy=-2., y=4.)])['stop'])

    def test_already_held_slow_or_stopped_stays_held(self):
        self.tick([self.obstacle()])
        for v in (1.9, .5, 0.):
            self.assertTrue(self.tick([self.obstacle(vx=v)])['stop'])

    def test_slow_clear_obstacle_releases_after_confirmation(self):
        self.tick([self.obstacle()])
        self.assertTrue(self.tick([self.obstacle(vx=0., y=4.)])['stop'])
        for _ in range(6):
            r = self.tick([self.obstacle(vx=0., y=4.)])
        self.assertFalse(r['stop'])
        self.assertEqual(r['waiting_ids'], [])

    def test_disappearance_does_not_release(self):
        self.tick([self.obstacle()])
        for _ in range(6):
            self.assertTrue(self.tick([])['stop'])

    def test_slow_second_obstacle_is_not_latched(self):
        r = self.tick([self.obstacle(), self.obstacle(vx=1., key=2)])
        self.assertEqual(r['waiting_ids'], [1])

    def test_outside_zone_unchanged(self):
        r = self.tick([self.obstacle()], x=20.)
        self.assertFalse(r['stop'])
        self.assertIsNone(r['cap_mps'])

    def test_missing_input_stop_preserved(self):
        r = self.g.step(self.t,self.t,(45.,0.),0.,None)
        self.assertTrue(r['stop'])
        self.assertEqual(r['reason'],'dynamic_obstacle_input_unavailable')

    def test_invalid_threshold_rejected(self):
        for value in (-1., float('nan'), float('inf'), True):
            with self.assertRaises(ValueError):
                DynamicZoneGuard(self.points, self.regions, min_obstacle_speed_mps=value)


if __name__ == '__main__':
    unittest.main()
