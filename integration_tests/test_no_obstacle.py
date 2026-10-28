import ast
from pathlib import Path
import unittest
import xml.etree.ElementTree as E
ROOT=Path(__file__).resolve().parents[1]
p=ROOT/'src/control/purepursuit_mgeo/scripts/no_obstacle_integration_node.py'
tree=ast.parse(p.read_text());fn=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='choose');ns={};exec(compile(ast.Module(body=[fn],type_ignores=[]),'choose','exec'),ns);choose=ns['choose']
class IntegrationTests(unittest.TestCase):
 def test_normal(self):
  self.assertEqual(choose(10,('nominal',10),({'active':False},10),None),('nominal','normal_highway'))
 def test_roundabout_exclusive(self):
  d={'active':True,'driver_input_fresh':True};self.assertEqual(choose(10,('nominal',10),({'active':True},10),(d,10)),(d,'roundabout_v61'))
 def test_stale_decision_stops(self):
  self.assertIsNone(choose(10,('nominal',10),({'active':False},9),None)[0])
 def test_no_nominal_fallback_in_roundabout(self):
  self.assertIsNone(choose(10,('nominal',10),({'active':True},10),None)[0])
 def test_exit_requires_fresh_nominal(self):
  self.assertIsNone(choose(10,('nominal',9),({'active':False},10),None)[0])
 def test_launch(self):
  r=E.parse(ROOT/'src/bringup/morai_bringup/launch/integrated_no_obstacle_v61.launch').getroot()
  self.assertEqual(r.find("arg[@name='morai_host_ip']").get('default'),'192.168.10.14')
  self.assertEqual(r.find("arg[@name='enable_control']").get('default'),'false')
  for n in r.findall('node'):self.assertNotIn('avoidance',n.get('type'))
  for n in r.findall('node'):
   self.assertTrue(list((ROOT/'src').rglob(n.get('type'))),n.get('type'))
if __name__=='__main__':unittest.main()
