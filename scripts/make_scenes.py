"""Fixed scene suite. No choices based on held-out evaluation results."""
import json
import xml.etree.ElementTree as ET
from pathlib import Path
start=[0,-.8,1.2,0,-.4,0]
goal=[1.,-.5,.8,.2,-.3,.1]
scenes=[
 ('open_sweep',start,goal,[]),
 ('open_reverse',goal,start,[]),
 ('center_column',start,goal,[([.32,.18,.28],[.10,.10,.45])]),
 ('low_bar',start,[1.4,-1.,1.4,0,-.4,0],[([.30,.25,.12],[.40,.10,.12])]),
 ('double_pillars',start,goal,[([.35,.17,.30],[.08,.08,.60]),([.25,.42,.30],[.08,.08,.60])]),
 ('narrow_passage',start,[1.5,-.6,1.,0,-.4,0],[([.35,.19,.30],[.12,.12,.60]),([.35,.43,.30],[.12,.12,.60])]),
 ('overhead_shelf',start,[1.2,-.7,1.,.5,-.3,0],[([.30,.28,.58],[.70,.50,.05])]),
 ('collision_goal',start,goal,[([.26,.40,.32],[.40,.40,.50])]),
 ('unreachable_joint_goal',start,[4.,-.5,.8,0,-.3,0],[]),
 ('blocked_start',start,goal,[([.20,0,.22],[.30,.20,.35])])]
Path('config/scenes').mkdir(parents=True,exist_ok=True)
for idx,(name,s,g,obs) in enumerate(scenes):
 d={'id':idx,'name':name,'start':s,'goal':g,'obstacles':[{'xyz':p,'size':z} for p,z in obs], 'budget_s':1.,'simulation_only':True}
 Path(f'config/scenes/{idx:02d}.json').write_text(json.dumps(d,indent=2)+'\n')
 w=ET.parse('models/world_with_arm.sdf');world=w.getroot().find('world')
 for j,(p,z) in enumerate(obs):
  m=ET.SubElement(world,'model',name=f'obstacle{j}');ET.SubElement(m,'static').text='true';ET.SubElement(m,'pose').text=' '.join(map(str,p))+' 0 0 0'
  l=ET.SubElement(m,'link',name='box')
  for kind in ['collision','visual']:
   c=ET.SubElement(l,kind,name=kind);ET.SubElement(ET.SubElement(ET.SubElement(c,'geometry'),'box'),'size').text=' '.join(map(str,z))
 w.write(f'config/scenes/{idx:02d}.world.sdf')
if __name__=='__main__':print('10 scene definitions written')
