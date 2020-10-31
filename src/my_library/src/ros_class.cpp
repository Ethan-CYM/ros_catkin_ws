#include <my_library/ros_class.h>

RosClass::RosClass(ros::NodeHandle *nodehandle):my_nh_(*nodehandle)
{
  ROS_INFO("calss constructor of RosClass");
  init_subscribers();
  init_publishers();
  init_services();
  num_remenber_ = 0.0;
}
void RosClass::init_subscribers()
{
  ROS_INFO("init Subscribers");
  my_subscriber_ = my_nh_.subscribe("class_in_topic", 1, &RosClass::sub_callback, this);
}

void RosClass::init_publishers()
{
  ROS_INFO("init publishers");
  my_publisher_ = my_nh_.advertise<std_msgs::Float32>("calss_out_topic", 1, true);
}

void RosClass::init_services()
{
  ROS_INFO("init services");
  my_service_ = my_nh_.advertiseService("service1", &RosClass::srv_callback, this);
}

void RosClass::sub_callback(const std_msgs::Float32 &message_holder)
{
  num_subscriber_ = message_holder.data;
  ROS_INFO("sub_callback attivated: rec val %f", num_subscriber_);
  std_msgs::Float32 output_msg;
  num_remenber_ += num_subscriber_;
  output_msg.data = num_remenber_;
  my_publisher_.publish(output_msg);
}

bool RosClass::srv_callback(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response)
{
  ROS_INFO("srv callback attivated");
  response.success = true;
  response.message = "class srv response";
  return true;
}
