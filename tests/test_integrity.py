import csv
import importlib.util
import json
import math
import sys
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
sys.path.insert(0,'scripts')
from report import audit,load

class Integrity(unittest.TestCase):
 def test_complete_evaluation(self):self.assertTrue(audit(load(Path('results/evaluation'))))
 def test_missing_rejected(self):
  with self.assertRaises(ValueError):audit([])
 def test_duplicate_rejected(self):
  rows=load(Path('results/evaluation'))
  with self.assertRaises(ValueError):audit(rows+[rows[0]])
 def test_model_geometry_matches(self):
  u=ET.parse('models/arm.urdf').getroot();s=ET.parse('models/arm.sdf').getroot().find('model')
  self.assertEqual(len([j for j in u.findall('joint') if j.get('type')=='revolute']),6)
  for i in range(1,7):
   sizeu=u.find(f"link[@name='link{i}']/collision/geometry/box").get('size').split()
   sizes=s.find(f"link[@name='link{i}']/collision/geometry/box/size").text.split()
   self.assertEqual(list(map(float,sizeu)),list(map(float,sizes)))
 def test_seed_split(self):
  m=json.loads(Path('results/evaluation/manifest.json').read_text());self.assertEqual(m['seeds'],list(range(20)))
  self.assertFalse(set(m['seeds'])&set(range(1000,1100)))
 def test_positive_control(self):
  d=json.loads(Path('results/control/10_RRTConnect_1004.json').read_text());self.assertTrue(d['tracking_pass']);self.assertTrue(d['execution_collision_valid'])
 def test_finite_and_recomputed_traces(self):
  for r in load(Path('results/evaluation')):
   if not r['execution_completed']:continue
   f=Path('results/evaluation')/f"{r['scene']:02d}_{r['planner']}_{r['seed']}.trace.csv"
   with f.open() as handle:rows=list(csv.DictReader(handle))
   errors=[]
   for x in rows:
    a=float(x['actual']);d=float(x['desired']);e=float(x['error']);v=float(x['velocity']);tau=float(x['torque'])
    self.assertTrue(all(math.isfinite(z) for z in [a,d,e,v,tau]))
    self.assertAlmostEqual(e,d-a,delta=2e-5);self.assertLessEqual(abs(tau),30.0001);errors.append(e)
   rmse=(sum(x*x for x in errors)/len(errors))**.5
   self.assertAlmostEqual(rmse,r['tracking_rmse_rad'],delta=2e-5)
 def test_frozen_eval_inputs_match(self):
  import hashlib
  m=json.loads(Path('results/evaluation/manifest.json').read_text())
  for p,h in m['inputs'].items():self.assertEqual(hashlib.sha256(Path(p).read_bytes()).hexdigest(),h)
 def test_failure_cases_preserved(self):
  r=load(Path('results/evaluation'));low=[x for x in r if x['scene']==3]
  self.assertEqual(len(low),40);self.assertTrue(all(not x['tracking_pass'] for x in low))
  self.assertEqual(sum(x['status']=='invalid_goal' for x in r),120)
if __name__=='__main__':unittest.main()
