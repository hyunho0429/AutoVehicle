import unittest
from types import SimpleNamespace as P
from purepursuit_mgeo.dynamic_zone_guard import DynamicZoneGuard


class WaitAndTunnelTests(unittest.TestCase):
    def setUp(self):
        self.g=DynamicZoneGuard([P(x=float(i),y=0.,z=0.) for i in range(201)],
            [dict(name='dynamic_obstacle',start_xyz=[40,0,0],end_xyz=[80,0,0])])
        self.t=100.
        self.path=[(45.,0.),(46.,3.),(60.,3.),(70.,0.)]

    def obj(self,key=1,vx=2.,x=50.,y=0.):
        return dict(id=key,center_x_map=x,center_y_map=y,length=1.,width=1.,
                    velocity_x_map=vx,velocity_y_map=0.,yaw=0.,yaw_valid=True,motion_state='MOVING')

    def tick(self,obs=None,speed=0.,x=45.,y=0.,tunnel=False,path=None):
        self.t+=.2
        obs=[self.obj()] if obs is None else obs
        sample=(dict(timestamp=self.t,obstacles=obs),self.t)
        return self.g.step(self.t,self.t,(x,y),speed,sample,
                           tunnel_mode=tunnel,detour_path=path)

    def wait(self):
        self.tick()
        for _ in range(26):r=self.tick([self.obj(vx=0.)])
        return r

    def test_five_seconds_requests_static_but_stays_stopped_without_plan(self):
        self.tick()
        for _ in range(24):r=self.tick([self.obj(vx=0.)])
        self.assertFalse(r['request_static_avoidance'])
        for _ in range(2):r=self.tick([self.obj(vx=0.)])
        self.assertTrue(r['request_static_avoidance'])
        self.assertTrue(r['stop'])
        self.assertEqual(r['reason'],'dynamic_static_plan_wait')

    def test_actual_vehicle_motion_resets_continuous_standstill(self):
        for _ in range(20):self.tick()
        self.tick(speed=.3)
        for _ in range(20):r=self.tick()
        self.assertFalse(r['request_static_avoidance'])

    def test_before_timeout_even_clear_detour_cannot_release(self):
        self.assertTrue(self.tick(path=self.path)['stop'])

    def test_safe_active_detour_releases_only_dynamic_stop(self):
        self.wait()
        r=self.tick([self.obj(vx=0.)],path=self.path)
        self.assertFalse(r['stop'])
        self.assertTrue(r['detour_driving'])
        self.assertEqual(r['waiting_ids'],[1])

    def test_original_path_is_not_a_safe_detour(self):
        self.wait()
        self.assertTrue(self.tick(path=[(45.,0.),(70.,0.)])['stop'])

    def test_other_static_obstacle_on_detour_blocks_release(self):
        self.wait()
        obs=[self.obj(vx=0.),self.obj(key=2,vx=0.,x=55.,y=3.)]
        self.assertTrue(self.tick(obs,path=self.path)['stop'])

    def test_new_fast_blocker_requires_new_wait(self):
        self.wait()
        r=self.tick([self.obj(vx=0.),self.obj(key=2,x=60.)],path=self.path)
        self.assertTrue(r['stop'])
        self.assertFalse(r['request_static_avoidance'])

    def test_detour_loss_reapplies_brake(self):
        self.wait();self.assertFalse(self.tick([self.obj(vx=0.)],path=self.path)['stop'])
        self.assertTrue(self.tick([self.obj(vx=0.)])['stop'])

    def test_passed_stationary_actor_can_finish_handoff(self):
        self.wait()
        for _ in range(6):r=self.tick([self.obj(vx=0.)],speed=2.,x=60.,y=3.,path=self.path)
        self.assertFalse(r['stop'])
        self.assertEqual(r['waiting_ids'],[])
        self.assertFalse(r['request_static_avoidance'])

    def test_gps_recovery_does_not_cancel_pending_wait(self):
        self.tick([self.obj(x=110.)],x=100.,tunnel=True)
        r=self.tick([self.obj(x=110.)],x=100.,tunnel=False)
        self.assertTrue(r['stop'])

    def test_tunnel_activates_dynamic_guard_outside_dynamic_zone(self):
        r=self.tick([self.obj(x=110.)],x=100.,tunnel=True)
        self.assertTrue(r['stop'])
        self.assertTrue(r['tunnel_dynamic_active'])

    def test_outside_zone_without_tunnel_unchanged(self):
        r=self.tick([self.obj(x=110.)],x=100.)
        self.assertFalse(r['stop'])
        self.assertIsNone(r['cap_mps'])

    def test_tunnel_still_uses_two_mps_threshold(self):
        r=self.tick([self.obj(x=110.,vx=1.99)],x=100.,tunnel=True)
        self.assertFalse(r['stop'])
        self.assertAlmostEqual(r['cap_mps'],20/3.6)

    def test_unmatched_tunnel_pose_does_not_authorize_driving(self):
        r=self.tick([self.obj(x=110.)],x=100.,y=20.,tunnel=True)
        self.assertTrue(r['stop'])
        self.assertEqual(r['reason'],'dynamic_route_unmatched')

    def test_missing_id_cannot_authorize_detour(self):
        self.wait()
        self.assertTrue(self.tick([],path=self.path)['stop'])

    def test_input_loss_does_not_count_toward_wait(self):
        for _ in range(20):self.tick()
        self.t+=1.
        self.g.step(self.t,self.t,(45.,0.),0.,None)
        for _ in range(20):r=self.tick()
        self.assertFalse(r['request_static_avoidance'])

    def test_unavailable_pose_does_not_count_toward_wait(self):
        for _ in range(20):self.tick()
        self.t+=1.
        self.g.step(self.t,self.t,None,0.,None)
        for _ in range(20):r=self.tick()
        self.assertFalse(r['request_static_avoidance'])


if __name__=='__main__':unittest.main()
