#include <ros/ros.h>
#include <my_library/ros_class.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "RosClassNode");
  ros::NodeHandle n;
  RosClass rosClass(&n);
  ros::spin();
  return 0;
}
