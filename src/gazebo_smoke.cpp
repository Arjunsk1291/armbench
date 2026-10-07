#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <sdf/sdf.hh>
#include <fstream>
#include <iterator>
#include <iostream>
std::string read_gz(const std::string& p) {std::ifstream f(p);return std::string(std::istreambuf_iterator<char>(f),{});}
void gazebo_smoke(){
  std::cerr<<"GZ setup start"<<std::endl;gazebo::setupServer();std::cerr<<"GZ setup done"<<std::endl;auto world=gazebo::loadWorld("models/world.sdf");std::cerr<<"GZ world loaded"<<std::endl;world->SetPaused(true);
  sdf::SDFPtr sdfmodel(new sdf::SDF());sdf::init(sdfmodel);sdf::readString(read_gz("models/arm.sdf"),sdfmodel);
  std::cerr<<"GZ insertion"<<std::endl;world->InsertModelSDF(*sdfmodel);world->Run();world->Step(1);std::cerr<<"GZ stepped"<<std::endl;
  auto robot=world->ModelByName("evidence_arm");
  if(!robot)throw std::runtime_error("Gazebo model spawn failed");
  for(int i=0;i<6;i++) {auto j=robot->GetJoint("joint"+std::to_string(i+1));std::cout<<"joint "<<i<<" "<<(j?j->Position(0):999)<<std::endl;}
  world->Step(100);std::cout<<"actual gazebo sim_time="<<world->SimTime().Double()<<std::endl;
  gazebo::shutdown();
}
