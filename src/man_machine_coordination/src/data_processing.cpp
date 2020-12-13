#include <ros/ros.h>
#include <unistd.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <std_msgs/Duration.h>

//关节角映射函数
float* jointAngleMap(float touchAngle[])
{

}

//Touch /phantom/joint_states 主题的消息回调函数
void AngleCallback(const sensor_msgs::JointState &message_holder)
{

}

//遥操作参数初始化
bool MMCoordinationInit(void)
{

}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "touch_teleoperation_ros");

	ros::NodeHandle n;

	ros::Subscriber touchAngleSub = n.subscribe("/phantom/joint_states",100,AngleCallback);
  ros::Publisher  urConParPub = n.advertise<trajectory_msgs::JointTrajectory>("ur_control_parameter", 1);

  MMCoordinationInit();

  while(ros::ok())
  {
    ros::spinOnce();
  }
	return 0;
}
