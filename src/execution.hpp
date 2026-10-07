#pragma once
#include <vector>
#include <string>
struct ExecutionResult {bool completed=false;double rmse=0,p95=0,max_error=0,sim_s=0;int samples=0;std::string error;};
ExecutionResult execute_gazebo(const std::string& worldfile,const std::vector<std::vector<double>>& path,const std::string& tracefile);
