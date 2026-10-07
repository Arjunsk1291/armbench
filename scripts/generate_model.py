"""Original procedural six-revolute-joint arm, identical collision boxes in URDF/SDF."""
import json
from pathlib import Path
import xml.etree.ElementTree as ET

LENGTHS=[.12,.28,.24,.10,.08,.06]
AXES=['0 0 1','0 1 0','0 1 0','1 0 0','0 1 0','1 0 0']
LIMITS=[(-2.8,2.8),(-1.7,1.7),(-2.1,2.1),(-2.8,2.8),(-2.0,2.0),(-2.8,2.8)]
WIDTH=.035

def generate(root):
    root=Path(root);root.mkdir(parents=True,exist_ok=True)
    urdf=['<robot name="evidence_arm">','<link name="world"/>',
          '<link name="base"><inertial><mass value="5"/><inertia ixx=".1" iyy=".1" izz=".1" ixy="0" ixz="0" iyz="0"/></inertial><visual><geometry><box size=".15 .15 .10"/></geometry></visual><collision><geometry><box size=".15 .15 .10"/></geometry></collision></link>',
          '<joint name="base_fixed" type="fixed"><parent link="world"/><child link="base"/><origin xyz="0 0 .05"/></joint>']
    sdf=['<sdf version="1.6"><model name="evidence_arm"><static>false</static>',
         '<link name="base"><pose>0 0 .05 0 0 0</pose><inertial><mass>5</mass><inertia><ixx>.1</ixx><iyy>.1</iyy><izz>.1</izz></inertia></inertial><collision name="base_collision"><geometry><box><size>.15 .15 .10</size></box></geometry></collision><visual name="base_visual"><geometry><box><size>.15 .15 .10</size></box></geometry></visual></link>',
         '<joint name="base_fixed" type="fixed"><parent>world</parent><child>base</child></joint>']
    cumulative=.0
    for i,(length,axis,limits) in enumerate(zip(LENGTHS,AXES,LIMITS)):
        parent='base' if i==0 else f'link{i}'
        link=f'link{i+1}';joint=f'joint{i+1}'
        offset='0 0 .08' if i==0 else f'{LENGTHS[i-1]} 0 0'
        mass=.35 if i<3 else .15
        iy=mass*(length**2+WIDTH**2)/12;ix=mass*(2*WIDTH**2)/12
        urdf += [f'<link name="{link}"><inertial><origin xyz="{length/2} 0 0"/><mass value="{mass}"/><inertia ixx="{ix}" iyy="{iy}" izz="{iy}" ixy="0" ixz="0" iyz="0"/></inertial>',
                 f'<visual><origin xyz="{length/2} 0 0"/><geometry><box size="{length} {WIDTH} {WIDTH}"/></geometry><material name="teal"><color rgba=".12 .6 .65 1"/></material></visual>',
                 f'<collision><origin xyz="{length/2} 0 0"/><geometry><box size="{length} {WIDTH} {WIDTH}"/></geometry></collision></link>',
                 f'<joint name="{joint}" type="revolute"><parent link="{parent}"/><child link="{link}"/><origin xyz="{offset}"/><axis xyz="{axis}"/><limit lower="{limits[0]}" upper="{limits[1]}" effort="30" velocity="1.5"/><dynamics damping=".2" friction="0"/></joint>']
        sdf += [f'<link name="{link}"><pose>{cumulative+length/2} 0 .13 0 0 0</pose><inertial><mass>{mass}</mass><inertia><ixx>{ix}</ixx><iyy>{iy}</iyy><izz>{iy}</izz></inertia></inertial>',
                f'<collision name="collision"><geometry><box><size>{length} {WIDTH} {WIDTH}</size></box></geometry></collision>',
                f'<visual name="visual"><geometry><box><size>{length} {WIDTH} {WIDTH}</size></box></geometry><material><ambient>.12 .6 .65 1</ambient></material></visual></link>',
                f'<joint name="{joint}" type="revolute"><parent>{parent}</parent><child>{link}</child><pose> {-length/2} 0 0 0 0 0</pose><axis><xyz>{axis}</xyz><use_parent_model_frame>true</use_parent_model_frame><limit><lower>{limits[0]}</lower><upper>{limits[1]}</upper><effort>30</effort><velocity>1.5</velocity></limit><dynamics><damping>.2</damping></dynamics></axis></joint>']
        cumulative+=length
    urdf+=['</robot>'];sdf+=['</model></sdf>']
    srdf=['<robot name="evidence_arm"><group name="arm"><chain base_link="base" tip_link="link6"/></group>']
    links=['base']+[f'link{i+1}' for i in range(6)]
    for a,b in zip(links,links[1:]):srdf.append(f'<disable_collisions link1="{a}" link2="{b}" reason="Adjacent"/>')
    srdf+=['</robot>']
    for f,lines in [('arm.urdf',urdf),('arm.sdf',sdf),('arm.srdf',srdf)]:
        text='\n'.join(lines)+'\n';ET.fromstring(text);(root/f).write_text(text)
    (root/'geometry.json').write_text(json.dumps({'lengths_m':LENGTHS,'width_m':WIDTH,'axes':AXES,'limits_rad':LIMITS,'simulation_only':True,'origin':'original procedural geometry, no external assets'},indent=2)+'\n')
if __name__=='__main__':generate('models')
