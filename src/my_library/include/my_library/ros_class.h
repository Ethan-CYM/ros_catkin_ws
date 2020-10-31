#ifndef ROS_CLASS_H_
#define ROS_CLASS_H_

#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_srvs/Trigger.h>

class RosClass
{
public:
  RosClass(ros::NodeHandle *nodehandle);
private:
  ros::NodeHandle my_nh_;
  ros::Subscriber my_subscriber_;
  ros::Publisher my_publisher_;
  ros::ServiceServer my_service_;

  double num_subscriber_;
  double num_remenber_;

  void init_subscribers();
  void init_publishers();
  void init_services();
  void sub_callback(const std_msgs::Float32 &message_holder);
  bool srv_callback(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response);
};

#endif
