"""Adapted upstream V1 numerical cases against the imported real scorer/helpers."""
import unittest, types
from purepursuit_mgeo.path import PathPoint
from purepursuit_mgeo import tunnel_frenet_planner as planner
from purepursuit_mgeo.tunnel_support import RecentThreatMemory as Memory, GpsFreshnessGate as Gate, planned_lateral_acceleration

def obstacle(track=1,x=20,y=0,length=2,width=1):
    return planner.TrackedObstacle(track,x,y,length,width,0,0,0,'STATIC',0,True,'BBOX')

def make_planner(tunnel=False):
    p=planner.FrenetPlanner(planner.ReferencePath([PathPoint(0,0,0),PathPoint(150,0,0)]),
        safety_margin=.3,maximum_lateral_acceleration=100,maximum_curvature=3,assume_static_obstacles=True)
    if tunnel:p.weight_offset=5.;p.weight_safety=40.;p.weight_clearance=100.;p.weight_completion=10.
    return p

OFFSETS=[1.,1.5,2.,2.5,3.,3.5,4.,4.5,5.,5.5,6.]

class Tests(unittest.TestCase):
    def test_wider_clearance_is_selected_in_tunnel(self):
        a=make_planner().plan(0,0,0,6,6,[obstacle()],0,OFFSETS,[2.,3.,4.])
        b=make_planner(True).plan(0,0,0,6,6,[obstacle()],0,OFFSETS,[2.,3.,4.])
        self.assertGreater(b.minimum_clearance,a.minimum_clearance)
        self.assertGreater(b.target_offset,a.target_offset)

    def test_wider_path_still_respects_walls(self):
        p=make_planner(True)
        objects=[obstacle(),obstacle(2,20,4.5,60,.2),obstacle(3,20,-4.5,60,.2)]
        c=p.plan(0,0,0,6,6,objects,0,OFFSETS,[2.,3.,4.])
        self.assertIsNotNone(c);self.assertLess(c.target_offset,4.)
        self.assertGreater(p.last_diagnostics['reasons'].get('collision',0),0)

    def test_only_threats_are_held_and_empty_frames_do_not_refresh_age(self):
        m=Memory(1.5);box=obstacle();wall=obstacle(2,20,5,20,.5)
        m.observe([box,wall],10);m.protect([1]);m.observe([],10.7)
        held,meta=m.supplement([],11.,True)
        self.assertEqual([o.track_id for o in held],[1]);self.assertEqual(meta,[(1,1.)])
        self.assertEqual(m.supplement([],11.51,True),([],[]))

    def test_redetection_updates_geometry_without_duplicate_same_id(self):
        m=Memory(1.5);m.observe([obstacle(width=.04)],10);m.protect([1])
        full=obstacle(width=1.4);m.observe([full],10.5)
        result,_=m.supplement([full],10.6,True)
        self.assertEqual(len(result),1);self.assertEqual(result[0].width,1.4)

    def test_planned_acceleration_continues_only_near_old_path(self):
        times=[i*.1 for i in range(11)]
        c=types.SimpleNamespace(points=[PathPoint(t,t*t,0) for t in times],times=times,
                                lateral_offsets=[t*t for t in times])
        self.assertAlmostEqual(planned_lateral_acceleration(c,(.5,.25,0,5),.946),2.)
        self.assertEqual(planned_lateral_acceleration(c,(20,0,0,5),.946),0.)

    def test_completion_cost_reduces_restarting_full_duration(self):
        p=make_planner(True)
        a=p.plan(0,0,0,6,6,[],0,[2.],[2.,3.,4.])
        b=p.plan(0,0,0,6,6,[],0,[2.],[2.,3.,4.],completion_budget=1.,continuation_direction=1)
        self.assertLess(b.planning_time,a.planning_time)

    def test_completion_preference_cannot_override_dynamics(self):
        p=make_planner(True);p.maximum_lateral_acceleration=1.
        c=p.plan(0,0,0,6,6,[],0,[2.],[2.,3.,4.],completion_budget=1.,continuation_direction=1)
        self.assertIsNotNone(c);self.assertEqual(c.planning_time,4.)

    def test_collision_is_still_a_hard_rejection(self):
        p=make_planner(True)
        self.assertIsNone(p.plan(0,0,0,6,6,[obstacle(x=1.35)],0,[-6.,6.],[2.,3.,4.]))

    def test_gps_gap_and_continuous_recovery(self):
        g=Gate(10);g.observe(10,10,0,0);self.assertFalse(g.observe(10.5,10,0,0))
        self.assertTrue(g.update(11.01));g.observe(12,12,0,0)
        self.assertTrue(g.update(12));g.observe(12.8,12.8,0,0)
        g.observe(13.1,13.1,0,0);self.assertFalse(g.update(13.1))


if __name__=='__main__': unittest.main()

