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
 RTDEReceiveInterface rtde_receive;
 public:
 UR_Data():rtde_receive("192.168.0.104")
 {

 }
  human_machine_coordination::RobotData urParmater;
  void get_urP(void)
  {
    std::vector<double> urP;
    urP = rtde_receive.getActualTCPPose();
    urParmater.pose.x = urP.at(0);
    urParmater.pose.y = urP.at(1);
    urParmater.pose.z = urP.at(2);
    urParmater.pose.Rx = urP.at(3);
    urParmater.pose.Ry = urP.at(4);
    urParmater.pose.Rz = urP.at(5);
//    printf("z = %2f \n",urP.at(0));
  }
};


int main(int argc, char **argv)
{
  UR_Data urData;

  ros::init(argc, argv, "hmc_ur_data");

  ros::NodeHandle n;

  ros::Publisher urDataPub = n.advertise<human_machine_coordination::RobotData>("hmc_ur_data", 1);

  ros::Rate loop_rate(128);

  while(ros::ok())
  {
    urData.get_urP();
    urData.urParmater.header.stamp = ros::Time::now();
    urDataPub.publish(urData.urParmater);
    loop_rate.sleep();
  }
  return 0;
}


