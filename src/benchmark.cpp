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
#include <json/json.h>
#include "execution.hpp"
#include <fstream>
#include <iostream>
#include <chrono>
#include <iterator>
#include <filesystem>
#include <cmath>
#include <sstream>
std::string read(const std::string& p){std::ifstream f(p);return std::string(std::istreambuf_iterator<char>(f),{});}
std::vector<double> vec(const Json::Value& a){std::vector<double>x;for(auto&v:a)x.push_back(v.asDouble());return x;}
Json::Value arr(const std::vector<double>& a){Json::Value x(Json::arrayValue);for(auto v:a)x.append(v);return x;}
int main(int argc,char**argv){
 if(argc<5){std::cerr<<"usage: armbench scene-index RRTConnect|PRM seed outputdir\n";return 2;}
 int sceneidx=std::stoi(argv[1]),seed=std::stoi(argv[3]);std::string algorithm=argv[2],outdir=argv[4];
 if(algorithm!="RRTConnect"&&algorithm!="PRM")return 2;
 char num[8];snprintf(num,sizeof(num),"%02d",sceneidx);std::string prefix="config/scenes/"+std::string(num);
 if(sceneidx==10)prefix="config/controls/positive_short";
 Json::Value cfg;std::ifstream cf(prefix+".json");cf>>cfg;
 auto qstart=vec(cfg["start"]),qgoal=vec(cfg["goal"]);
 if(qstart.size()!=6||qgoal.size()!=6)throw std::runtime_error("scene missing");
 std::filesystem::create_directories(outdir);std::string stem=outdir+"/"+num+"_"+algorithm+"_"+std::to_string(seed);
 Json::Value result;result["scene"]=sceneidx;result["scene_name"]=cfg["name"];result["planner"]=algorithm;result["seed"]=seed;result["budget_s"]=1.;result["simulation_only"]=true;
 result["planning_success"]=false;result["collision_valid"]=false;result["execution_completed"]=false;
 auto whole=std::chrono::steady_clock::now();rclcpp::init(argc,argv);
 auto node=rclcpp::Node::make_shared("arm_benchmark");auto ud=urdf::parseURDF(read("models/arm.urdf"));auto sd=std::make_shared<srdf::Model>();sd->initString(*ud,read("models/arm.srdf"));
 auto model=std::make_shared<moveit::core::RobotModel>(ud,sd);auto scene=std::make_shared<planning_scene::PlanningScene>(model);auto group=model->getJointModelGroup("arm");
 for(Json::ArrayIndex i=0;i<cfg["obstacles"].size();i++){
  auto o=cfg["obstacles"][i];moveit_msgs::msg::CollisionObject obj;obj.header.frame_id="world";obj.id="obstacle"+std::to_string(i);obj.operation=obj.ADD;
  shape_msgs::msg::SolidPrimitive box;box.type=box.BOX;for(auto v:vec(o["size"]))box.dimensions.push_back(v);obj.primitives.push_back(box);
  geometry_msgs::msg::Pose pose;pose.position.x=o["xyz"][0].asDouble();pose.position.y=o["xyz"][1].asDouble();pose.position.z=o["xyz"][2].asDouble();pose.orientation.w=1.;obj.primitive_poses.push_back(pose);scene->processCollisionObjectMsg(obj);
 }
 moveit::core::RobotState start(model),goal(model);start.setToDefaultValues();goal.setToDefaultValues();start.setJointGroupPositions(group,qstart);goal.setJointGroupPositions(group,qgoal);start.update();goal.update();
 bool startvalid=start.satisfiesBounds(group)&&!scene->isStateColliding(start,"arm");bool goalvalid=goal.satisfiesBounds(group)&&!scene->isStateColliding(goal,"arm");
 result["start_valid"]=startvalid;result["goal_valid"]=goalvalid;
 auto t=std::chrono::steady_clock::now();
 if(!startvalid||!goalvalid){result["status"]=!startvalid?"invalid_start":"invalid_goal";result["planning_wall_s"]=0.;result["planner_reported_s"]=0.;}
 else {
  ompl::RNG::setSeed(seed+1);
  pluginlib::ClassLoader<planning_interface::PlannerManager> loader("moveit_core","planning_interface::PlannerManager");auto planner=loader.createSharedInstance("ompl_interface/OMPLPlanner");planner->initialize(model,node,"ompl");
  planning_interface::PlannerConfigurationSettings ps;ps.name="arm["+algorithm+"]";ps.group="arm";ps.config["type"]="geometric::"+algorithm;ps.config["longest_valid_segment_fraction"]="0.005";
  if(algorithm=="RRTConnect")ps.config["range"]="0.0";else ps.config["max_nearest_neighbors"]="10";
  planning_interface::PlannerConfigurationMap map;map[ps.name]=ps;auto def=ps;def.name="arm";map[def.name]=def;planner->setPlannerConfigurations(map);
  planning_interface::MotionPlanRequest req;req.group_name="arm";req.planner_id=algorithm;req.allowed_planning_time=1.;req.num_planning_attempts=1;
  moveit::core::robotStateToRobotStateMsg(start,req.start_state);req.goal_constraints.push_back(kinematic_constraints::constructGoalConstraints(goal,group,1e-5,1e-5));
  moveit_msgs::msg::MoveItErrorCodes code;auto ctx=planner->getPlanningContext(scene,req,code);planning_interface::MotionPlanResponse res;
  t=std::chrono::steady_clock::now();bool ok=ctx&&ctx->solve(res);
  result["planning_wall_s"]=std::chrono::duration<double>(std::chrono::steady_clock::now()-t).count();result["planner_reported_s"]=res.planning_time_;result["moveit_code"]=res.error_code_.val;
  result["status"]=ok?"success":"timeout_or_planner_failure";result["planning_success"]=ok;
  if(ok&&res.trajectory_){
   std::vector<std::vector<double>> path;double length=0;bool valid=true;int checked=0;
   for(size_t i=0;i<res.trajectory_->getWayPointCount();i++){
    std::vector<double> q;res.trajectory_->getWayPoint(i).copyJointGroupPositions(group,q);path.push_back(q);
    if(i>0){double sq=0,mx=0;for(int j=0;j<6;j++){double dq=q[j]-path[i-1][j];sq+=dq*dq;mx=std::max(mx,std::abs(dq));}length+=std::sqrt(sq);
     int n=std::max(1,int(std::ceil(mx/.01)));for(int k=0;k<=n;k++){moveit::core::RobotState state(model);std::vector<double> interp(6);for(int j=0;j<6;j++)interp[j]=path[i-1][j]+double(k)/n*(q[j]-path[i-1][j]);state.setJointGroupPositions(group,interp);state.update();checked++;if(!state.satisfiesBounds(group)||scene->isStateColliding(state,"arm"))valid=false;}
    }
   }
   result["path_length_rad"]=length;result["waypoints"]=int(path.size());result["collision_valid"]=valid;result["collision_checked_states"]=checked;
   Json::Value trajectory(Json::arrayValue);for(auto&q:path)trajectory.append(arr(q));std::ofstream(stem+".path.json")<<trajectory;
   if(!valid)result["status"]="invalid_path";
   else {
    auto et=std::chrono::steady_clock::now();auto ex=execute_gazebo(prefix+".world.sdf",path,stem+".trace.csv");
    result["execution_wall_s"]=std::chrono::duration<double>(std::chrono::steady_clock::now()-et).count();result["execution_completed"]=ex.completed;result["tracking_rmse_rad"]=ex.rmse;result["tracking_p95_rad"]=ex.p95;result["tracking_max_rad"]=ex.max_error;result["execution_sim_s"]=ex.sim_s;result["tracking_samples"]=ex.samples;result["execution_error"]=ex.error;
    result["tracking_pass"]=ex.completed&&ex.rmse<=.15;
    std::ifstream trace(stem+".trace.csv");std::string line;std::getline(trace,line);std::vector<double> actual(6);int actual_bad=0,actual_checked=0;
    while(std::getline(trace,line)){std::stringstream ls(line);std::string cell;std::vector<std::string> cols;while(std::getline(ls,cell,','))cols.push_back(cell);if(cols.size()!=7)continue;int j=std::stoi(cols[1]);actual[j]=std::stod(cols[3]);if(j==5){moveit::core::RobotState st(model);st.setJointGroupPositions(group,actual);st.update();actual_checked++;if(!st.satisfiesBounds(group)||scene->isStateColliding(st,"arm"))actual_bad++;}}
    result["actual_state_checked_samples"]=actual_checked;result["actual_invalid_samples"]=actual_bad;result["execution_collision_valid"]=ex.completed&&actual_bad==0;

   }
  }
 }
 result["process_wall_s"]=std::chrono::duration<double>(std::chrono::steady_clock::now()-whole).count();result["status_label"]="MEASURED";
 std::ofstream(stem+".json")<<result<<'\n';std::cout<<result<<std::endl;rclcpp::shutdown();return 0;
}
