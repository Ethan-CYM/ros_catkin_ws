#include <ros/ros.h>
#include <cym_nodes/ServiceMsg.h>
#include <iostream>
#include <string>
using namespace std;

bool callback(cym_nodes::ServiceMsg::Request &request, cym_nodes::ServiceMsg::Response &response)
{
  ROS_INFO("callback activated");
  string in_name(request.name);
  response.on_the_list = false;

  if(in_name.compare("Bob") == 0){
    ROS_INFO("asked abort Bob");
    response.age = 32;
    response.good_guy = false;
    response.on_the_list = true;
    response.nickname = "Terrible";
  }
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "service");
  ros::NodeHandle n;
  ros::ServiceServer service = n.advertiseService("lookup_by_name",callback);
  ROS_INFO("ready to lookup_by_name");
  ros::spin();
  return 0;
}
