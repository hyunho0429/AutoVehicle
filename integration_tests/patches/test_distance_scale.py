import math
import unittest
from ekf_local_enu.distance_scale import DistanceScale


class DistanceScaleTests(unittest.TestCase):
    def feed(self, learner, until=6., ratio=.84, start=0., gps=True, vx=4., vy=0.):
        for i in range(round(start*100)+1, round(until*100)+1):
            t=i/100
            learner.add_motion(t,vx*.01,vy*.01,.01)
            if gps and i%10==0:
                learner.observe_gps(t,ratio*vx*t,ratio*vy*t)
        return learner

    def test_learns_observed_ratio(self):
        g=self.feed(DistanceScale())
        self.assertTrue(g.ready)
        self.assertAlmostEqual(g.factor,.84,places=6)

    def test_correct_input_stays_unscaled(self):
        g=self.feed(DistanceScale(),ratio=1.)
        self.assertTrue(g.ready)
        self.assertAlmostEqual(g.factor,1.)

    def test_uses_both_axes(self):
        g=self.feed(DistanceScale(),vx=2.4,vy=3.2)
        self.assertAlmostEqual(g.factor,.84)

    def test_freezes_when_gps_disappears(self):
        g=self.feed(DistanceScale());factor=g.factor
        self.feed(g,until=15.,start=6.,gps=False)
        self.assertEqual(g.factor,factor)
        self.assertEqual(g.reason,'gps_gap_frozen')

    def test_does_not_train_across_gps_gap_or_recovery_jump(self):
        g=self.feed(DistanceScale());factor=g.factor
        self.feed(g,until=9.,start=6.,gps=False)
        g.observe_gps(9.,400.,400.)
        g.add_motion(9.01,.04,0.,.01)
        self.assertEqual(g.factor,factor)
        self.assertEqual(len(g.candidates),0)

    def test_initial_gps_absence_does_not_invent_scale(self):
        g=self.feed(DistanceScale(),gps=False)
        self.assertFalse(g.ready)
        self.assertEqual(g.factor,1.)

    def test_standstill_never_calibrates(self):
        g=self.feed(DistanceScale(),vx=0.)
        self.assertFalse(g.ready)

    def test_disabled_is_identity(self):
        g=self.feed(DistanceScale(False))
        self.assertEqual(g.factor,1.)
        self.assertFalse(g.ready)

    def test_bad_gps_direction_rejected(self):
        g=self.feed(DistanceScale(),ratio=-.84)
        self.assertFalse(g.ready)
        self.assertGreater(g.rejected,0)

    def test_bad_gps_scale_rejected(self):
        g=self.feed(DistanceScale(),ratio=3.)
        self.assertFalse(g.ready)

    def test_stale_speed_breaks_training_and_preserves_learned_factor(self):
        g=self.feed(DistanceScale());factor=g.factor
        g.add_motion(6.01,.04,0.,.01,valid=False)
        self.assertEqual(g.factor,factor)
        self.assertEqual(g.fixes,[])

    def test_pending_future_gps_waits_for_motion(self):
        g=DistanceScale()
        g.add_motion(.1,.4,0.,.1)
        g.observe_gps(.15,.504,0.)
        self.assertEqual(len(g.pending),1)
        g.add_motion(.2,.4,0.,.1)
        self.assertEqual(len(g.pending),0)
        self.assertAlmostEqual(g.fixes[-1][3],.6)

    def test_duplicate_fixes_do_not_count_as_new_evidence(self):
        g=self.feed(DistanceScale(),until=2.5)
        count=len(g.candidates)
        for _ in range(100):g.observe_gps(2.5,.84*4*2.5,0.)
        self.assertEqual(len(g.candidates),count)

    def test_curved_or_returning_motion_not_used(self):
        g=DistanceScale()
        for i in range(1,601):
            t=i/100;w=2.*math.pi
            dx=math.cos(w*t)*.04;dy=math.sin(w*t)*.04
            g.add_motion(t,dx,dy,.01)
            if i%10==0:g.observe_gps(t,.84*4/w*math.sin(w*t),.84*4/w*(1-math.cos(w*t)))
        self.assertFalse(g.ready)


if __name__=='__main__':unittest.main()
