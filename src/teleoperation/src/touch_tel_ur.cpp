#include <ros/ros.h>
#include <unistd.h>
#include <sensor_msgs/JointState.h>
#include <omni_msgs/OmniButtonEvent.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <std_msgs/Duration.h>

//映射系数，最大为1，值越大，ur关节角范围越大
float mapCoefficient = 0.5;
//抽样率，touch 话题位姿消息的频率为1000，ur不必每个点都跟随
float sampleRate = 0.08;
//延迟时间,单位s，延迟时间越长，每次发给ur的轨迹点越多，ur运行越顺滑，但有动作延迟感。
float defelayTime = 0.4;
//Touch消息频率,一般都为1000，不要改动
float touchMsgHz = 1000;

float PI = 3.14;

//全局变量定义
trajectory_msgs::JointTrajectory urMsg;
trajectory_msgs::JointTrajectoryPoint trajectoryPoint;

bool pubFlag = false;
bool newDataFrameFlag = true;

int seq;
int sampleNum;
int maxSampleNum;
int lastSampleSecs;
int lastSampleNsecs;
int fristDataSecs;
int fristDataNsecs;
int sampleIntevalNsecs;
int minSampleIntevalNsecs;
int pubIntevalNsecs;
float urTrajectoryPointTime;

int greyButton;
int whiteButton;

//关节角映射函数
float* jointAngleMap(float touchAngle[])
{
  int i;
  static float urAngle[6];

  urAngle[0] = touchAngle[0] * mapCoefficient - 2 * PI / 4;
  urAngle[1] = (touchAngle[1] - 0.2689) * mapCoefficient - 5 * PI / 9;
  urAngle[2] = (touchAngle[2] + 0.6397) * mapCoefficient - 7 * PI / 9;
  urAngle[3] = -(touchAngle[4] + PI) * mapCoefficient - 0 * PI / 4;
  urAngle[4] = (touchAngle[3] - PI) * mapCoefficient * 0.5 + 2 * PI / 4;
  urAngle[5] = (touchAngle[5] + PI) * mapCoefficient * 0.5 - 0 * PI / 4;

  return urAngle;
}

//Touch /phantom/joint_states 主题的消息回调函数
void AngleCallback(const sensor_msgs::JointState &message_holder)
{
  int i;
  float touchAngle[6];
  float* urAngle;

  bool sampleDataFlag = false;

  if(newDataFrameFlag == true){ //为真时候代表写新的数据帧
    newDataFrameFlag = false;
    urTrajectoryPointTime = 0;
    lastSampleSecs = message_holder.header.stamp.sec;
    lastSampleNsecs = message_holder.header.stamp.nsec;
    fristDataSecs = message_holder.header.stamp.sec;
    fristDataNsecs =  message_holder.header.stamp.nsec;
    urMsg.points.clear();
    urMsg.header.seq = ++seq;
    urMsg.header.stamp = ros::Time::now();
    urMsg.header.frame_id = "ur_control_parameter";
    sampleDataFlag = true;
  }
  else{ //判断是否采集此次Touch的数据
    sampleIntevalNsecs = (message_holder.header.stamp.sec - lastSampleSecs)* 1000000000 + message_holder.header.stamp.nsec - lastSampleNsecs;
    if(sampleIntevalNsecs >= minSampleIntevalNsecs){
      lastSampleSecs = message_holder.header.stamp.sec;
      lastSampleNsecs = message_holder.header.stamp.nsec;
      sampleDataFlag = true;
    }
  }

  //采集Touch关节角数据，并写入当前数据帧
  if(sampleDataFlag == true){
    for(i = 0; i < 6 ;i++){
      touchAngle[i] = message_holder.position[i];
    }
    urAngle = jointAngleMap(touchAngle);

    urTrajectoryPointTime += (float)minSampleIntevalNsecs/1000000000.0;
    trajectoryPoint.positions.clear();
    trajectoryPoint.positions.push_back(urAngle[0]);
    trajectoryPoint.positions.push_back(urAngle[1]);
    trajectoryPoint.positions.push_back(urAngle[2]);
    trajectoryPoint.positions.push_back(urAngle[3]);
    trajectoryPoint.positions.push_back(urAngle[4]);
    trajectoryPoint.positions.push_back(urAngle[5]);
    trajectoryPoint.time_from_start = ros::Duration(urTrajectoryPointTime);
    urMsg.points.push_back(trajectoryPoint);
    sampleNum += 1;
    //判断数据帧是否完成
    if(sampleNum >= maxSampleNum){
      pubIntevalNsecs = (message_holder.header.stamp.sec - fristDataSecs)* 1000000000 + message_holder.header.stamp.nsec - fristDataNsecs;
      sampleNum = 0;
      pubFlag = true;
    }
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
  if (defelayTime < 0.001/sampleRate && defelayTime > 1.0) {
    ROS_INFO("延迟时间设置错误，参考范围(0.002/sampleRate) ~ 1.0 当前值为: [%f]", defelayTime);
  }
  else{
    maxSampleNum = (int)touchMsgHz*defelayTime*sampleRate;
    minSampleIntevalNsecs = (int)(1000000000.0/(sampleRate*touchMsgHz));
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
      newDataFrameFlag = true;
    }
    ros::spinOnce();
  }
	return 0;
}
