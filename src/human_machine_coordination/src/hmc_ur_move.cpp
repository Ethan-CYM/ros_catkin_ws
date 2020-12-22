#include <unistd.h>
#include <thread>
#include <chrono>
#include <ros/ros.h>
#include <hmc_ur_move.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <ur_rtde/rtde_control_interface.h>
#include <human_machine_coordination/RobotControl.h>

#define pi 3.1415926
using namespace ur_rtde;
using namespace std::chrono;

class  UR_Control{
 private:
  RTDEControlInterface rtdeControl;
 public:
  std::vector<double> targetPose;
  Move_Arg moveArg;
 UR_Control(std::string IPAdress = "127.0.0.1"):rtdeControl(IPAdress)
 {

 }
 void moveInit(void)
 {
     moveArg.initJoint = {-(45.0/180)*pi, -(100.0/180)*pi, -(90.0/180)*pi, -(80.0/180)*pi, (90.0/180)*pi, (45.0/180)*pi};

     rtdeControl.moveJ(moveArg.initJoint, moveArg.velocity, moveArg.acceleration);
 }
 void urContralCallback(const human_machine_coordination::RobotControl &msg)
 {
   targetPose.clear();
   targetPose.push_back(msg.pose.x);
   targetPose.push_back(msg.pose.y);
   targetPose.push_back(msg.pose.z);
   targetPose.push_back(msg.pose.Rx);
   targetPose.push_back(msg.pose.Ry);
   targetPose.push_back(msg.pose.Rz);
 }

 void servolmove(void)
 {
   auto t_start = high_resolution_clock::now();
   rtdeControl.servoL(targetPose,
                      moveArg.servol.velocity,
                      moveArg.servol.acceleration,
                      moveArg.invertalTime,
                      moveArg.servol.lookaheadTime,
                      moveArg.servol.gain);
   auto t_stop = high_resolution_clock::now();
   auto t_duration = std::chrono::duration<double>(t_stop - t_start);
   if (t_duration.count() < moveArg.invertalTime)
   {
     std::this_thread::sleep_for(std::chrono::duration<double>(moveArg.invertalTime - t_duration.count()));
   }
 }

 //force control mode, the function is blocked
 void forceMode(void)
 {
   auto t_start = high_resolution_clock::now();
   rtdeControl.forceMode(moveArg.force.task_frame,
                         moveArg.force.selection_vector,
                         moveArg.force.wrench,
                         moveArg.force.force_type,
                         moveArg.force.limitsSpeed);
   auto t_stop = high_resolution_clock::now();
   auto t_duration = std::chrono::duration<double>(t_stop - t_start);
   if (t_duration.count() < moveArg.invertalTime)
   {
     std::this_thread::sleep_for(std::chrono::duration<double>(moveArg.invertalTime - t_duration.count()));
   }
 }

// void forceMode

 ~UR_Control()
{
   if(moveArg.mode == servol){
     rtdeControl.servoStop();
   }
   else if(moveArg.mode == force){
     rtdeControl.forceModeStop();
   }
  rtdeControl.stopScript();
  delete &rtdeControl;
}
};



int main(int argc, char* argv[])
{
    UR_Control urR("192.168.0.104");
    ros::init(argc, argv, "hmc_ur_move");
    ros::NodeHandle n;
    ros::Subscriber URMoveSub = n.subscribe("hmc_ur_control", 100, &UR_Control::urContralCallback, &urR);
    urR.moveInit();
    printf("ur have moved to init pose\n");

    ros::AsyncSpinner AS(1);
    AS.start();

//    usleep(200000);

    while(ros::ok())
    {
      urR.servolmove();
//      urR.forceMode();
    }
    delete &urR;
    return 0;
}
