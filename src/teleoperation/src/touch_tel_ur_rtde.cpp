#include <ros/ros.h>
#include <unistd.h>
#include <sensor_msgs/JointState.h>
#include <omni_msgs/OmniButtonEvent.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <std_msgs/Duration.h>

//映射系数，最大为1，值越大，ur关节角范围越大
float mapCoefficient = 1;
//抽样率，touch 话题位姿消息的频率为1000，ur不必每个点都跟随
float sampleRate = 0.5;
//Touch消息频率,一般都为1000，不要改动
float touchMsgHz = 1000;

float PI = 3.14;

//全局变量定义
trajectory_msgs::JointTrajectory urMsg;
trajectory_msgs::JointTrajectoryPoint trajectoryPoint;

bool pubFlag = false;

int seq;
int sampleIntevalNum;
int getTouchMsgNum;

int greyButton;
int whiteButton;

//关节角映射函数
float* jointAngleMap(float touchAngle[])
{
  int i;
  static float urAngle[6];
//安全的位姿：
  // urAngle[0] = touchAngle[0] * mapCoefficient - 2 * PI / 4;
  // urAngle[1] = (touchAngle[1] - 0.2689) * mapCoefficient - 2 * PI / 3;
  // urAngle[2] = (touchAngle[2] + 0.6397) * mapCoefficient - 2 * PI / 3;
  // urAngle[3] = (touchAngle[4] + PI) * mapCoefficient - 2 * PI / 4;
  // urAngle[4] = (touchAngle[3] - PI) * mapCoefficient * 0.5 + 2 * PI / 4;
  // urAngle[5] = (touchAngle[5] + PI) * mapCoefficient * 0.5 - 0 * PI / 4;

  urAngle[0] = -(touchAngle[0] * mapCoefficient) * 1.25 + 2 * PI / 4;
  urAngle[1] = -(touchAngle[1] - 0.2689) * mapCoefficient * 1.25 - 3.5 * PI / 9;
  urAngle[2] = -(touchAngle[2] + 0.6397) * mapCoefficient + 2 * PI / 3;
  urAngle[3] = (touchAngle[4] + PI) * mapCoefficient - PI;
  urAngle[4] = -(touchAngle[3] - PI) * mapCoefficient * 0.5 - 2 * PI / 4;
  urAngle[5] = -(touchAngle[5] + PI) * mapCoefficient * 0.8 - 0 * PI / 4;

  return urAngle;
}

//Touch /phantom/joint_states 主题的消息回调函数
void AngleCallback(const sensor_msgs::JointState &message_holder)
{
  int i;
  float touchAngle[6];
  float* urAngle;

  //采集Touch关节角数据，并写入当前数据帧
  if(getTouchMsgNum == 0){
    for(i = 0; i < 6 ;i++){
      touchAngle[i] = message_holder.position[i];
    }
    urAngle = jointAngleMap(touchAngle);
    urMsg.points.clear();
    trajectoryPoint.positions.clear();
    trajectoryPoint.positions.push_back(urAngle[0]);
    trajectoryPoint.positions.push_back(urAngle[1]);
    trajectoryPoint.positions.push_back(urAngle[2]);
    trajectoryPoint.positions.push_back(urAngle[3]);
    trajectoryPoint.positions.push_back(urAngle[4]);
    trajectoryPoint.positions.push_back(urAngle[5]);
    urMsg.points.push_back(trajectoryPoint);
  }
  getTouchMsgNum++;
  if(getTouchMsgNum >= sampleIntevalNum){
      pubFlag = true;
  }
}

//Touch /phantom/button 主题的（按钮）消息订阅器的回调函数
void ButtonCallback(const omni_msgs::OmniButtonEvent &message_holder)
{
  /*
  ROS_INFO("received Header is: [%i]", message_holder.grey_button);

  ROS_INFO("received name is: [%i]", message_holder.white_button);
  */
}

//遥操作参数初始化
bool TeleoperationInit(void)
{
  if (mapCoefficient < 0 && mapCoefficient > 1.0) {
    ROS_INFO("映射系数设置错误，参考范围0~1.0 当前值为: [%f]", mapCoefficient);
  }

  if (sampleRate < 0 && sampleRate > 1.0) {
    ROS_INFO("采样率设置错误，参考范围0~1.0 当前值为: [%f]", sampleRate);
  }
  else{
    sampleIntevalNum = (int)(1.0/sampleRate);
  }
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "touch_teleoperation_ros");

	ros::NodeHandle n;

	ros::Subscriber touchAngleSub = n.subscribe("/phantom/joint_states",100,AngleCallback);

  ros::Subscriber touchButtonSub = n.subscribe("/phantom/button",10,ButtonCallback);

  ros::Publisher  urConParPub = n.advertise<trajectory_msgs::JointTrajectory>("ur_control_parameter", 1);

  TeleoperationInit();

  while(ros::ok())
  {
    if(pubFlag == true){
      pubFlag = false;
      //ROS_INFO("sampleNum is: [%i]", sampleNum);
      urConParPub.publish(urMsg);
      getTouchMsgNum = 0;
    }
    ros::spinOnce();
  }
	return 0;
}
