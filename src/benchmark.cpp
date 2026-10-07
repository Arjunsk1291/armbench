#include <rclcpp/rclcpp.hpp>
#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_state/conversions.h>
#include <pluginlib/class_loader.hpp>
#include <ompl/util/RandomNumbers.h>
#include <fstream>
#include <iostream>
#include <chrono>
#include <iterator>

std::string read(const std::string& p) {std::ifstream f(p);return std::string(std::istreambuf_iterator<char>(f),{});}
void gazebo_smoke();
int main(int argc,char**argv) {
  rclcpp::init(argc,argv);
  auto node=rclcpp::Node::make_shared("arm_benchmark");
  auto ud=urdf::parseURDF(read("models/arm.urdf"));
  auto sd=std::make_shared<srdf::Model>();sd->initString(*ud,read("models/arm.srdf"));
  auto model=std::make_shared<moveit::core::RobotModel>(ud,sd);
  auto scene=std::make_shared<planning_scene::PlanningScene>(model);
  auto group=model->getJointModelGroup("arm");
  moveit::core::RobotState start(model),goal(model);
  start.setToDefaultValues();goal.setToDefaultValues();
  start.setJointGroupPositions(group,std::vector<double>{0,-.8,1.2,0,-.4,0});start.update();
  goal.setJointGroupPositions(group,std::vector<double>{1.,-.5,.8,.2,-.3,.1});goal.update();
  std::cout<<"start collision="<<scene->isStateColliding(start,"arm")<<" goal="<<scene->isStateColliding(goal,"arm")<<std::endl;
  ompl::RNG::setSeed(1000);
  pluginlib::ClassLoader<planning_interface::PlannerManager> loader("moveit_core","planning_interface::PlannerManager");
  auto planner=loader.createSharedInstance("ompl_interface/OMPLPlanner");
  planner->initialize(model,node,"ompl");
  planning_interface::PlannerConfigurationSettings ps;ps.name="arm[RRTConnect]";ps.group="arm";ps.config["type"]="geometric::RRTConnect";
  ps.config["range"]="0.0";ps.config["longest_valid_segment_fraction"]="0.005";
  planning_interface::PlannerConfigurationMap map;map[ps.name]=ps;
  auto def=ps;def.name="arm";map[def.name]=def;planner->setPlannerConfigurations(map);
  planning_interface::MotionPlanRequest req;req.group_name="arm";req.planner_id="RRTConnect";req.allowed_planning_time=1.;req.num_planning_attempts=1;
  moveit::core::robotStateToRobotStateMsg(start,req.start_state);
  req.goal_constraints.push_back(kinematic_constraints::constructGoalConstraints(goal,group));
  moveit_msgs::msg::MoveItErrorCodes code;auto ctx=planner->getPlanningContext(scene,req,code);
  planning_interface::MotionPlanResponse res;
  auto t=std::chrono::steady_clock::now();bool ok=ctx&&ctx->solve(res);
  std::cout<<"plan ok="<<ok<<" code="<<res.error_code_.val<<" wall="<<std::chrono::duration<double>(std::chrono::steady_clock::now()-t).count()<<" waypoints="<<(res.trajectory_?res.trajectory_->getWayPointCount():0)<<std::endl;
  gazebo_smoke();rclcpp::shutdown();return ok?0:2;
}
