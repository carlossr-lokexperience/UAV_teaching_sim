"""Pruebas rapidas. Ejecutar: python -m unittest discover -s tests -v"""
import ast
import io
import json
import math
import pathlib
import unittest
from dataclasses import asdict, replace
from unittest.mock import patch
import numpy as np
import config as C
from aircraft import Aircraft, State, Actuation, wrap_pi
from guidance import project_on_path, point_ahead, pure_pursuit, vector_field, GuidanceCommand
from control import Controller, PID
from commands import parse_rules, validate_command, call_llm
from navigation import Navigation
from perception import image_to_ground, detect_road, PathObservation
from autopilot import Autopilot
from world import World, Camera

class GeometryTests(unittest.TestCase):
    def setUp(self):
        self.path=np.array([[0.,0.],[0.,100.],[0.,200.]])
        self.state=State(x=10,y=20,heading=0,vx=0,vy=20)
        self.settings=C.Settings()
    def test_projection_on_segment(self):
        i,t,p,d,e=project_on_path((10,25),self.path)
        self.assertEqual(i,0); self.assertAlmostEqual(t,.25); self.assertAlmostEqual(e,10)
        np.testing.assert_allclose(p,[0,25])
    def test_point_ahead(self):
        np.testing.assert_allclose(point_ahead(self.path,0,.5,80),[0,130])
    def test_endpoint(self):
        np.testing.assert_allclose(point_ahead(self.path,1,.8,80),[0,200])
    def test_closed_wrap(self):
        path=np.array([[0,0],[0,10],[10,10],[10,0],[0,0]],float)
        np.testing.assert_allclose(point_ahead(path,3,.5,10),[0,5])
    def test_repeated_points(self):
        with self.assertRaises(ValueError): project_on_path((0,0),[[0,0],[0,0]])
    def test_reject_nan_path(self):
        with self.assertRaises(ValueError): project_on_path((0,0),[[0,0],[0,np.nan]])
    def test_pursuit_left(self):
        self.assertLess(pure_pursuit(self.state,self.path,self.settings).bank_rad,0)
    def test_pursuit_right(self):
        self.state.x=-10
        self.assertGreater(pure_pursuit(self.state,self.path,self.settings).bank_rad,0)
    def test_pursuit_straight(self):
        self.state.x=0
        self.assertAlmostEqual(pure_pursuit(self.state,self.path,self.settings).bank_rad,0)
    def test_vector_left(self):
        self.assertLess(vector_field(self.state,self.path,self.settings).bank_rad,0)
    def test_wrap_boundary(self):
        self.assertAlmostEqual(wrap_pi(math.radians(359)),math.radians(-1))

class ControlTests(unittest.TestCase):
    def test_bank_saturation(self):
        c=Controller(); cfg=C.Settings(bank_limit_deg=20)
        c.update(State(),GuidanceCommand(math.radians(75),100,20),.02,cfg)
        self.assertAlmostEqual(math.degrees(c.bank_reference),20)
    def test_rate_limit(self):
        a=Controller().update(State(),GuidanceCommand(2,220,32),.02,C.Settings())
        self.assertLessEqual(abs(a.roll_rate),math.radians(C.MAX_ROLL_RATE_DEG_S))
    def test_positive_bank_turns_right(self):
        a=Aircraft(State(bank=.2)); a.update_physics(Actuation(),.02)
        self.assertGreater(a.state.heading,0)
    def test_speed_envelope(self):
        a=Aircraft(State(airspeed=32)); a.update_physics(Actuation(acceleration=999),.1)
        self.assertLessEqual(a.state.airspeed,32)
    def test_invalid_dt(self):
        with self.assertRaises(ValueError): Aircraft(State()).update_physics(Actuation(),1)
    def test_nan_actuation(self):
        with self.assertRaises(ValueError): Aircraft(State()).update_physics(Actuation(roll_rate=float('nan')),.02)
    def test_wind_drift(self):
        a=Aircraft(State()); a.update_physics(Actuation(),.1,(4,0))
        self.assertAlmostEqual(a.state.x,.4)
    def test_pid_antiwindup(self):
        p=PID(1,1,0,1)
        for _ in range(1000): p.update(100,.02)
        self.assertEqual(p.integral,0)
    def test_altitude_converges(self):
        cfg=C.Settings(altitude_m=120); a=Aircraft(State()); ctrl=Controller()
        for _ in range(2000):
            ref=GuidanceCommand(0,120,20); a.update_physics(ctrl.update(a.state,ref,.02,cfg),.02)
        self.assertAlmostEqual(a.state.altitude,120,places=2)

class CommandTests(unittest.TestCase):
    def setUp(self):
        self.cfg=C.Settings(); self.state=State(heading=math.radians(40))
    def test_absolute_height(self):
        self.assertEqual(parse_rules('sube a 120 metros'),{'altitude_m':120})
    def test_relative_height(self):
        self.assertEqual(parse_rules('baja 20 metros'),{'altitude_delta_m':-20})
    def test_speed_relative(self):
        self.assertEqual(parse_rules('reduce velocidad 3'),{'speed_delta_mps':-3})
    def test_speed_absolute(self):
        self.assertEqual(parse_rules('reduce velocidad a 18'),{'speed_mps':18})
    def test_turn_and_heading_mode(self):
        c=validate_command(parse_rules('gira 30 grados a la izquierda'),self.cfg,self.state)
        self.assertAlmostEqual(math.degrees(c.heading_rad),10); self.assertEqual(c.mode,'HEADING')
    def test_camera_compound(self):
        c=parse_rules('sigue la carretera con camara a 120 metros y a 18 m/s')
        self.assertEqual(c,{'mode':'FOLLOW_ROAD','source':'CAMERA','altitude_m':120,'speed_mps':18})
    def test_accent(self):
        self.assertEqual(parse_rules('anticipaci\u00f3n 90'),{'lookahead_m':90})
    def test_vector_field(self):
        self.assertEqual(parse_rules('usa vector field'),{'guidance':'VECTOR_FIELD'})
    def test_no_orbit(self):
        with self.assertRaises(ValueError): parse_rules('orbita sobre la torre')
    def test_unknown(self):
        with self.assertRaises(ValueError): parse_rules('haz magia')
    def test_reject_extreme_speed(self):
        with self.assertRaises(ValueError): validate_command({'speed_mps':100},self.cfg,self.state)
        self.assertEqual(self.cfg.speed_mps,20)
    def test_atomic_command(self):
        before=asdict(self.cfg)
        with self.assertRaises(ValueError): validate_command({'altitude_m':150,'speed_mps':1000},self.cfg,self.state)
        self.assertEqual(before,asdict(self.cfg))
    def test_boolean_not_number(self):
        with self.assertRaises(ValueError): validate_command({'speed_mps':True},self.cfg,self.state)
    def test_nan_rejected(self):
        with self.assertRaises(ValueError): validate_command({'speed_mps':float('nan')},self.cfg,self.state)
    def test_extra_fields_rejected(self):
        with self.assertRaises(ValueError): validate_command({'exec':'print(1)'},self.cfg,self.state)
    def test_conflicting_fields(self):
        with self.assertRaises(ValueError): validate_command({'altitude_m':100,'altitude_delta_m':20},self.cfg,self.state)
    def test_heading_follow_conflict(self):
        with self.assertRaises(ValueError): validate_command({'mode':'FOLLOW_ROAD','heading_deg':90},self.cfg,self.state)
    def test_llm_http_payload_and_result(self):
        def opener(request,timeout):
            data=json.loads(request.data)
            self.assertFalse(data['stream']); self.assertIn('format',data)
            self.assertTrue(request.full_url.endswith('/api/chat'))
            return io.BytesIO(json.dumps({'message':{'content':'{"source":"CAMERA"}'}}).encode())
        with patch.object(C,'LLM_MODEL','mock-only-not-a-model'):
            self.assertEqual(call_llm('usa la camara',self.cfg,opener),{'source':'CAMERA'})
    def test_llm_empty_model(self):
        with patch.object(C,'LLM_MODEL',''):
            with self.assertRaises(ValueError): call_llm('hola',self.cfg)
    def test_llm_invalid_json(self):
        with patch.object(C,'LLM_MODEL','mock'):
            with self.assertRaises(ValueError): call_llm('hola',self.cfg,lambda *a,**k:io.BytesIO(b'not-json'))

class VisionAndNavigationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls): cls.world=World()
    def test_pixels_center_is_position(self):
        s=State(x=30,y=40)
        np.testing.assert_allclose(image_to_ground([[160,160]],s,320,320),[[30,40]])
    def test_pixel_right_north_heading(self):
        s=State()
        p=image_to_ground([[200,160]],s,320,320)[0]
        self.assertGreater(p[0],0); self.assertAlmostEqual(p[1],0)
    def test_projection_roundtrip(self):
        for h in [0,.5,1.5,3.0]:
            s=State(x=70,y=-30,heading=h)
            xy=np.array([[60,-20],[75,-25],[40,-50]])
            uv,depth=Camera.nadir(s).project(np.column_stack((xy,np.zeros(3))))
            np.testing.assert_allclose(image_to_ground(uv,s,320,320),xy,atol=1e-7)
    def test_vision_from_real_render(self):
        s=self.world.initial_state(); rgb=self.world.render_downward(s)
        o=detect_road(rgb,s)
        self.assertTrue(o.valid); self.assertGreater(o.confidence,.8)
        for p in o.points:
            self.assertLess(abs(project_on_path(p,self.world.road)[4]),2)
    def test_blank_image_loses_road(self):
        o=detect_road(np.zeros((320,320,3),np.uint8),State())
        self.assertFalse(o.valid)
    def test_camera_never_uses_map(self):
        class ForbiddenMap:
            def __array__(self,*args,**kwargs): raise AssertionError('Fuga de mapa')
        cfg=C.Settings(source='CAMERA'); ap=Autopilot(cfg)
        s=State(); o=PathObservation(np.array([[0,-100],[0,0],[0,100],[0,200]]),1,0)
        ap.update(s,ForbiddenMap(),o)
        self.assertEqual(cfg.mode,'FOLLOW_ROAD')
    def test_stale_camera_fallback(self):
        cfg=C.Settings(source='CAMERA'); ap=Autopilot(cfg)
        o=PathObservation(np.array([[0,-100],[0,0],[0,100],[0,200]]),1,0)
        ap.update(State(),None,o); ref=ap.update(State(time=2),None,o)
        self.assertEqual(cfg.mode,'HEADING'); self.assertIn('CAMARA',ref.status)
    def test_no_gnss_timeout(self):
        nav=Navigation(); e=None
        for i in range(400): e=nav.estimate_state(State(time=i*.02),.02,'PERFECT',False)
        self.assertFalse(e.valid); self.assertGreater(e.fix_age,5)
    def test_nav_loss_fallback(self):
        cfg=C.Settings(); ap=Autopilot(cfg)
        ap.update(State(valid=False),self.world.road)
        self.assertEqual(cfg.mode,'HEADING')
    def test_truth_is_copied(self):
        truth=State(); e=Navigation().estimate_state(truth,.02)
        e.x=100
        self.assertEqual(truth.x,0)
    def test_state_filter_is_finite(self):
        nav=Navigation()
        for i in range(500): e=nav.estimate_state(State(time=i*.02,x=i*.02*20,vx=20,vy=0),.02,'FILTERED')
        self.assertTrue(np.isfinite([e.x,e.y,e.vx,e.vy]).all())
    def test_python39_syntax(self):
        root=pathlib.Path(__file__).resolve().parents[1]
        for p in root.glob('*.py'):
            ast.parse(p.read_text(encoding='utf-8'),feature_version=(3,9))
    def test_no_orbit_mode(self):
        with self.assertRaises(ValueError): C.Settings(mode='ORBIT').validate()

if __name__=='__main__': unittest.main()
