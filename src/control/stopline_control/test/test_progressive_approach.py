"""Approach timing and clearance checks with an ideal pedal plant (not MORAI)."""

import os
import sys
import unittest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src")))
from stopline_control.core import AccelRiseLimiter, StopLineControllerCore


def approach(initial_kph=35.0, distance=60.0, planning_decel=1.2, brake_gain=1.5):
    core = StopLineControllerCore(
        front_reference_offset_m=3.845, hold_distance_m=0.5,
        planning_decel_mps2=planning_decel, settle_distance_m=0.15)
    speed, dt = initial_kph / 3.6, 0.05
    limiter = AccelRiseLimiter(rate_per_sec=0.5)
    history = []
    for tick in range(2400):
        now = 100.0 + tick * dt
        core.observe_line(distance + 3.845, 1.0, True, now, now, now,
                          target_id="mapped-stop")
        core.observe_signal("RED", 1.0, True, now, now, now)
        decision = core.update(now, now, speed)
        history.append((tick * dt, distance, speed * 3.6, decision))
        if decision.mode == "HOLD" and speed <= 0.01:
            break
        nominal_accel = min(1.0, max(0.0, 35.0 / 3.6 - speed))
        pedal = 0.0 if decision.brake else min(nominal_accel, decision.accel_limit)
        pedal = limiter.limit(pedal, decision.brake, now, now)
        following = max(0.0, speed + (1.4 * pedal - brake_gain * decision.brake) * dt)
        distance -= 0.5 * (speed + following) * dt
        speed = following
    return history


class ProgressiveApproachTest(unittest.TestCase):
    def test_remote_red_does_not_brake_a_car_below_the_distance_curve(self):
        core = StopLineControllerCore(planning_decel_mps2=1.2)
        core.observe_line(45.0, 1.0, True, 10.0, 10.0, 10.0)
        core.observe_signal("RED", 1.0, True, 10.0, 10.0, 10.0)
        decision = core.update(10.0, 10.0, 30.0 / 3.6)
        self.assertEqual(decision.brake, 0.0)
        self.assertGreater(decision.accel_limit, 0.0)

    def test_approach_does_not_crawl_for_metres(self):
        history = approach()
        self.assertTrue(all(speed > 10.0 for _, distance, speed, _ in history
                            if 5.0 <= distance <= 15.0))
        self.assertFalse(any(speed <= 3.0 and distance > 1.2
                             for _, distance, speed, _ in history))
        self.assertLess(history[-1][0], 20.0)
        self.assertEqual(history[-1][3].mode, "HOLD")
        self.assertAlmostEqual(history[-1][1], 0.5, delta=0.1)

    def test_low_initial_speed_can_recover_before_final_deceleration(self):
        history = approach(initial_kph=2.0, distance=30.0)
        self.assertGreater(max(speed for _, distance, speed, _ in history
                               if distance > 10.0), 15.0)
        self.assertEqual(history[-1][3].mode, "HOLD")
        self.assertAlmostEqual(history[-1][1], 0.5, delta=0.1)

    def test_high_speed_and_stronger_brakes_keep_clearance(self):
        for initial, gain in ((55.0, 1.5), (35.0, 3.0)):
            with self.subTest(initial=initial, gain=gain):
                history = approach(initial_kph=initial, distance=110.0, brake_gain=gain)
                self.assertEqual(history[-1][3].mode, "HOLD")
                self.assertAlmostEqual(history[-1][1], 0.5, delta=0.1)

    def test_late_red_uses_full_brake_when_stopping_distance_is_exhausted(self):
        core = StopLineControllerCore(planning_decel_mps2=1.2)
        core.observe_line(8.0, 1.0, True, 10.0, 10.0, 10.0)
        core.observe_signal("RED", 1.0, True, 10.0, 10.0, 10.0)
        self.assertEqual(core.update(10.0, 10.0, 35.0 / 3.6).brake, 1.0)


if __name__ == "__main__":
    unittest.main()
