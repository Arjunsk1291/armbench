"""One fresh process per OMPL seed. Serial, resumable, integrity-checked."""
import argparse
import hashlib
import json
import os
import platform
import subprocess
import time
from pathlib import Path

def frozen_inputs():
    files=sorted([*Path('models').glob('*'),*Path('config/scenes').glob('*'),*Path('src').glob('*'),Path('conda-linux-64.lock'),Path('config/PREREGISTRATION.md')])
    return {str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in files if p.is_file()}

def main():
    p=argparse.ArgumentParser();p.add_argument('--dev',action='store_true');p.add_argument('--max-seconds',type=float,default=0);a=p.parse_args()
    out=Path('results/development' if a.dev else 'results/evaluation');out.mkdir(parents=True,exist_ok=True)
    mf=out/'manifest.json';manifest={'schema':1,'simulation_only':True,'inputs':frozen_inputs(),'seeds':list(range(1000,1002)) if a.dev else list(range(20)),'planners':['RRTConnect','PRM'],'scene_ids':list(range(10)),'budget_s':1.0,'tracking':'torque-limited PD, original synthetic inertias (minimum .005 kg m2), max .5 rad/s command'}
    if mf.exists() and json.loads(mf.read_text())!=manifest:raise ValueError('Frozen inputs changed; do not mix runs')
    mf.write_text(json.dumps(manifest,indent=2)+'\n')
    hw=out/'hardware.json'
    if not hw.exists():hw.write_text(json.dumps({'os':platform.platform(),'cpu':subprocess.check_output(['lscpu'],text=True),'commit':subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),'simulation_only':True},indent=2)+'\n')
    begun=time.monotonic();done=0
    for scene in range(10):
      for planner in manifest['planners']:
       for seed in manifest['seeds']:
        stem=out/f'{scene:02d}_{planner}_{seed}'
        if stem.with_suffix('.json').exists():done+=1;continue
        if a.max_seconds and time.monotonic()-begun>=a.max_seconds:print('checkpoint',done);return
        cmd=['./build/armbench',str(scene),planner,str(seed),str(out)]
        env={**os.environ,'GAZEBO_MODEL_DATABASE_URI':''}
        with stem.with_suffix('.log').open('w') as log:
         try:r=subprocess.run(cmd,env=env,stdout=log,stderr=subprocess.STDOUT,timeout=30)
         except subprocess.TimeoutExpired:
          raise RuntimeError(f'Process timeout {stem}, no fake result written')
        if r.returncode!=0 or not stem.with_suffix('.json').exists():raise RuntimeError(f'Failed {stem}, see log')
        d=json.loads(stem.with_suffix('.json').read_text());assert (d['scene'],d['planner'],d['seed'])==(scene,planner,seed)
        done+=1;print(done,scene,planner,seed,d['status'],flush=True)
    print('DONE',done)
if __name__=='__main__':main()
