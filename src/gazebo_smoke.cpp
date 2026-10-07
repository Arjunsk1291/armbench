#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <sdf/sdf.hh>
#include <fstream>
#include <iterator>
#include <iostream>
std::string read_gz(const std::string& p) {std::ifstream f(p);return std::string(std::istreambuf_iterator<char>(f),{});}
void gazebo_smoke(){
  gazebo::setupServer();auto world=gazebo::loadWorld("models/world.sdf");world->SetPaused(true);
  sdf::SDFPtr sdfmodel(new sdf::SDF());sdf::init(sdfmodel);sdf::readString(read_gz("models/arm.sdf"),sdfmodel);
  world->InsertModelSDF(*sdfmodel);world->Step(1);
  auto robot=world->ModelByName("evidence_arm");
  if(!robot)throw std::runtime_error("Gazebo model spawn failed");
  for(int i=0;i<6;i++) {auto j=robot->GetJoint("joint"+std::to_string(i+1));std::cout<<"joint "<<i<<" "<<(j?j->Position(0):999)<<std::endl;}
  world->Step(100);std::cout<<"actual gazebo sim_time="<<world->SimTime().Double()<<std::endl;
  gazebo::shutdown();
}
