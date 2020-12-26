#include <thread>
#include <chrono>
#include <ros/ros.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <ur_rtde/rtde_control_interface.h>
#include <human_machine_coordination/RobotData.h>


using namespace ur_rtde;
using namespace std::chrono;

class  UR_Data{
 private:
 RTDEReceiveInterface rtdeReceive;
 human_machine_coordination::RobotData urData;
 public:
 UR_Data(std::string IPAdress = "127.0.0.1"):rtdeReceive(IPAdress)
 {

 }
  human_machine_coordination::RobotData get_Parmater(void)
  {

    urData.actualTCPPose = rtdeReceive.getActualTCPPose();
    urData.targetTCPPose = rtdeReceive.getTargetTCPPose();
    urData.actualTCPSpeed = rtdeReceive.getActualTCPSpeed();
    urData.targetTCPSpeed = rtdeReceive.getTargetTCPSpeed();
    urData.actualTCPForce = rtdeReceive.getActualTCPForce();
    urData.actualToolAccel = rtdeReceive.getActualToolAccelerometer();
    return urData;
  }
  void publishMsg(ros::Publisher publisher, human_machine_coordination::RobotData msg)
  {
    msg.header.stamp = ros::Time::now();
    publisher.publish(msg);
  }
};


int main(int argc, char **argv)
{
  UR_Data urRight("192.168.0.104");

  ros::init(argc, argv, "hmc_ur_data");

  ros::NodeHandle n;

  ros::Publisher urDataPub = n.advertise<human_machine_coordination::RobotData>("hmc_ur_data", 1);

  ros::Rate loop_rate(128);

  while(ros::ok())
  {
    urRight.publishMsg(urDataPub, urRight.get_Parmater());
    loop_rate.sleep();
  }
  return 0;
}


