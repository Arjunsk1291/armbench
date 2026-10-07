"""Strict complete-dataset report; execution metrics never mix with planner timing."""
import csv
import json
import math
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

def load(root):
 return [json.loads(p.read_text()) for p in root.glob('[0-9][0-9]_*.json') if '.path.' not in p.name]
def audit(rows,seeds=range(20)):
 expected={(i,p,s) for i in range(10) for p in ['RRTConnect','PRM'] for s in seeds}
 keys=[(r['scene'],r['planner'],r['seed']) for r in rows]
 if len(set(keys))!=len(keys):raise ValueError('Duplicate keys')
 if set(keys)!=expected:raise ValueError(f'Incomplete dataset: {len(rows)}/{len(expected)}')
 for r in rows:
  assert r['simulation_only'] and r['budget_s']==1.
  if r['status']=='success':assert r['planning_success'] and r['collision_valid']
  for k in ['planning_wall_s','process_wall_s']:
   if not math.isfinite(r[k]) or r[k]<0:raise ValueError('Invalid timing')
  if r['execution_completed']:
   for k in ['tracking_rmse_rad','tracking_p95_rad','tracking_max_rad']:
    if not math.isfinite(r[k]):raise ValueError('Non-finite execution metric')
 return True

def stats(x):
 return {'n':len(x),'median':float(np.median(x)),'p95':float(np.percentile(x,95))} if x else {'n':0,'median':None,'p95':None}
def main():
 root=Path('results/evaluation');rows=load(root);audit(rows);out=Path('report');out.mkdir(exist_ok=True)
 summary={'simulation_only':True,'status':'MEASURED','runs':len(rows),'planners':{},'per_scene':{}}
 for p in ['RRTConnect','PRM']:
  rr=[r for r in rows if r['planner']==p];valid=[r for r in rr if r['status']=='success'];executed=[r for r in valid if r['execution_completed']]
  statuses={s:sum(r['status']==s for r in rr) for s in sorted({r['status'] for r in rr})}
  summary['planners'][p]={'attempts':len(rr),'statuses':statuses,'planning_returned_success':sum(r['planning_success'] for r in rr),'dense_valid_plans':len(valid),'execution_completed':len(executed),'tracking_pass':sum(r.get('tracking_pass',False) for r in rr),'execution_sampled_valid':sum(r.get('execution_collision_valid',False) for r in rr),
   'all_attempt_planning_s':stats([r['planning_wall_s'] for r in rr]),'valid_request_planning_s':stats([r['planning_wall_s'] for r in rr if r['start_valid'] and r['goal_valid']]),'successful_plan_s':stats([r['planning_wall_s'] for r in valid]),'path_length_rad':stats([r['path_length_rad'] for r in valid]),'tracking_rmse_rad':stats([r['tracking_rmse_rad'] for r in executed]),'tracking_p95_rad':stats([r['tracking_p95_rad'] for r in executed])}
 for i in range(10):
  rr=[r for r in rows if r['scene']==i];summary['per_scene'][str(i)]={'name':rr[0]['scene_name'],**{p:{'valid_plans':sum(r['status']=='success' for r in rr if r['planner']==p),'tracking_pass':sum(r.get('tracking_pass',False) for r in rr if r['planner']==p),'execution_sampled_valid':sum(r.get('execution_collision_valid',False) for r in rr if r['planner']==p)} for p in ['RRTConnect','PRM']}}
 (out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n');(out/'raw.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in sorted(rows,key=lambda r:(r['scene'],r['planner'],r['seed']))))
 keys=sorted(set().union(*(r.keys() for r in rows)))
 with (out/'raw.csv').open('w',newline='') as f:w=csv.DictWriter(f,keys);w.writeheader();w.writerows(rows)
 fig,ax=plt.subplots(figsize=(10,5));x=np.arange(10);width=.36
 for off,p,col in [(-.18,'RRTConnect','#167d95'),(.18,'PRM','#b46d21')]:
  ax.bar(x+off,[summary['per_scene'][str(i)][p]['valid_plans'] for i in range(10)],width,label=p,color=col)
 ax.set_xticks(x,[summary['per_scene'][str(i)]['name'] for i in range(10)],rotation=35,ha='right');ax.set_ylim(0,20);ax.set_ylabel('Dense-valid plans / 20 seeds');ax.set_title('MoveIt + Gazebo simulation: all fixed scenes, including failures');ax.legend();fig.tight_layout();fig.savefig(out/'success.png',dpi=160);plt.close(fig)
 fig,axs=plt.subplots(1,2,figsize=(10,4))
 for ax,key,label in [(axs[0],'planning_wall_s','Planning wall time (s), valid requests'),(axs[1],'tracking_rmse_rad','Gazebo joint tracking RMSE (rad)')]:
  groups=[[r[key] for r in rows if r['planner']==p and (r['start_valid'] and r['goal_valid'] if key=='planning_wall_s' else r['execution_completed'])] for p in ['RRTConnect','PRM']]
  ax.boxplot(groups,tick_labels=['RRTConnect','PRM'],showfliers=True);ax.set_ylabel(label)
 fig.suptitle('Simulation: planning and execution measured separately');fig.tight_layout();fig.savefig(out/'timing_tracking.png',dpi=160);plt.close(fig)
 print(json.dumps(summary['planners'],indent=2))
if __name__=='__main__':main()
