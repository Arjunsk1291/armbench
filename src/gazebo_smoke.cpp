#include "execution.hpp"
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <fstream>
#include <algorithm>
#include <cmath>
#include <iostream>

ExecutionResult execute_gazebo(const std::string& worldfile,const std::vector<std::vector<double>>& path,const std::string& tracefile){
 ExecutionResult out;
 if(path.size()<2){out.error="no trajectory";return out;}
 gazebo::setupServer();auto world=gazebo::loadWorld(worldfile);world->SetPaused(true);world->Run();
 auto robot=world->ModelByName("evidence_arm");if(!robot)throw std::runtime_error("model missing");
 std::vector<gazebo::physics::JointPtr> joints;
 for(int j=0;j<6;j++){auto joint=robot->GetJoint("joint"+std::to_string(j+1));joint->SetPosition(0,path[0][j]);joint->SetVelocity(0,0);joints.push_back(joint);}
 std::ofstream tr(tracefile);tr<<"sim_t,joint,desired,actual,velocity,torque,error\n";
 std::vector<double> errors;
 double time=0;int step=0;double sumsq=0;
 const double dt=.001,torque_limit=30.;
 const double kp[6]={30,30,20,3,2,1},kd[6]={2,2,1,.04,.03,.01};
 auto hold=[&](const std::vector<double>& desired){
  for(int j=0;j<6;j++){
   double actual=joints[j]->Position(0),vel=joints[j]->GetVelocity(0),err=desired[j]-actual;
   double tau=std::clamp(kp[j]*err-kd[j]*vel,-torque_limit,torque_limit);joints[j]->SetForce(0,tau);
   if(step%10==0){tr<<time<<','<<j<<','<<desired[j]<<','<<actual<<','<<vel<<','<<tau<<','<<err<<'\n';errors.push_back(std::abs(err));sumsq+=err*err;}
   if(!std::isfinite(actual)||std::abs(actual)>10)throw std::runtime_error("dynamics divergence");
  }
  world->Step(1);step++;time+=dt;
 };
 try {
  for(int k=0;k<500;k++)hold(path[0]);
  for(size_t i=1;i<path.size();i++){
   double maxdq=0;for(int j=0;j<6;j++)maxdq=std::max(maxdq,std::abs(path[i][j]-path[i-1][j]));
   int n=std::max(1,int(std::ceil(std::max(.05,maxdq/.5)/dt)));
   for(int k=1;k<=n;k++){std::vector<double> q(6);for(int j=0;j<6;j++)q[j]=path[i-1][j]+(path[i][j]-path[i-1][j])*double(k)/n;hold(q);}
  }
  for(int k=0;k<500;k++)hold(path.back());out.completed=true;
 }catch(const std::exception&e){out.error=e.what();}
 out.sim_s=time;out.samples=errors.size();out.rmse=std::sqrt(sumsq/std::max(1,out.samples));
 if(!errors.empty()){std::sort(errors.begin(),errors.end());out.max_error=errors.back();out.p95=errors[std::min(errors.size()-1,size_t(std::ceil(errors.size()*.95)-1))];}
 world->Stop();gazebo::shutdown();return out;
}
