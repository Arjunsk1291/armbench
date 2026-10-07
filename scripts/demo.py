"""Reconstruct actual Gazebo joint-state traces, not a fresh renderer/physics run."""
import csv
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import imageio.v2 as imageio

L=np.array([.12,.28,.24,.10,.08,.06]);axes=['z','y','y','x','y','x']
def rotation(axis,q):
 c,s=np.cos(q),np.sin(q)
 if axis=='x':return np.array([[1,0,0],[0,c,-s],[0,s,c]])
 if axis=='y':return np.array([[c,0,s],[0,1,0],[-s,0,c]])
 return np.array([[c,-s,0],[s,c,0],[0,0,1]])
def fk(q):
 t=np.array([0.,0.,.13]);r=np.eye(3);points=[t.copy()]
 for i in range(6):
  r=r@rotation(axes[i],q[i]);t=t+r@np.array([L[i],0,0]);points.append(t.copy())
 return np.array(points)
def readtrace(path):
 rows=list(csv.DictReader(path.open()));samples=[]
 for i in range(0,len(rows),6):
  block=rows[i:i+6]
  if len(block)==6:samples.append((float(block[0]['sim_t']),[float(x['actual']) for x in block],[float(x['desired']) for x in block]))
 return samples

def main():
 out=Path('report');out.mkdir(exist_ok=True)
 # Fixed scenes 0 and 3, seed0, BOTH planners: easy control plus known failure retained.
 seq=[]
 for scene in [0,3]:
  a=readtrace(Path(f'results/evaluation/{scene:02d}_RRTConnect_0.trace.csv'));b=readtrace(Path(f'results/evaluation/{scene:02d}_PRM_0.trace.csv'))
  n=max(len(a),len(b))
  for k in range(0,n,7):seq.append((scene,a[min(k,len(a)-1)],b[min(k,len(b)-1)]))
 fig=plt.figure(figsize=(10,5),dpi=100);axs=[fig.add_subplot(1,2,j+1,projection='3d') for j in range(2)]
 with imageio.get_writer(out/'demo.mp4',fps=15,codec='libx264',macro_block_size=2,ffmpeg_params=['-crf','23','-pix_fmt','yuv420p']) as w:
  for scene,a,b in seq:
   for ax,p,trace in zip(axs,['RRTConnect','PRM'],[a,b]):
    ax.clear();t,actual,desired=trace;pa,pd=fk(actual),fk(desired)
    ax.plot(pd[:,0],pd[:,1],pd[:,2],'--',color='#b46d21',label='commanded');ax.plot(pa[:,0],pa[:,1],pa[:,2],'o-',color='#167d95',lw=4,label='Gazebo actual')
    if scene==3:
     # Obstacle box projected as transparent surfaces.
     x,y,z=.30,.25,.12;sx,sy,sz=.40,.10,.12
     xx,yy=np.meshgrid([x-sx/2,x+sx/2],[y-sy/2,y+sy/2]);ax.plot_surface(xx,yy,np.full_like(xx,z+sz/2),alpha=.25,color='red')
    ax.set_xlim(-.1,.9);ax.set_ylim(-.2,.9);ax.set_zlim(0,.8);ax.set_xlabel('x (m)');ax.set_ylabel('y (m)');ax.set_zlabel('z (m)');ax.view_init(25,45)
    ax.set_title(f'{p} | scene {scene} | seed 0 | t={t:.2f}s');ax.legend(loc='upper right',fontsize=7)
   fig.suptitle('Simulation: actual Gazebo trace reconstruction, not hardware',fontsize=12)
   fig.texts=[fig.texts[0]]
   fig.text(.02,.02,'Scene 0: positive control. Scene 3: tracking/collision failure. Full results include every attempt.',fontsize=9)
   fig.tight_layout(rect=(0,.05,1,.94));fig.canvas.draw();w.append_data(np.asarray(fig.canvas.buffer_rgba())[:,:,:3])
 plt.close(fig)
 (out/'demo.json').write_text(json.dumps({'simulation_only':True,'source':'actual stored Gazebo joint-state traces, forward-kinematics visualization','scenes':[0,3],'seed':0,'planners':['RRTConnect','PRM'],'not_live_renderer':True},indent=2)+'\n')
if __name__=='__main__':main()
