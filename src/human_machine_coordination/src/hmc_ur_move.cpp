#include <unistd.h>
#include <thread>
#include <chrono>
#include <math.h>
#include <ros/ros.h>
#include <hmc_ur_move.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <ur_rtde/rtde_control_interface.h>
#include <human_machine_coordination/RobotControl.h>
#include <data_processing.h>

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
 //移动到初始位置
 void moveInit(void)
 {
     moveArg.initPose = {0.3, -0.45, 0.3, 0.0, 3.14, 0.0};
     targetPose = moveArg.initPose;
     rtdeControl.moveJ_IK(moveArg.initPose, moveArg.velocity, moveArg.acceleration);
 }
 //机器人末端位置的位置消息回调函数
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
 //伺服移动
 void servolMove(void)
 {
   double servolTime = 1.0 / CTRL_FREQ;
   auto t_start = high_resolution_clock::now();
   rtdeControl.servoL(targetPose,
                      moveArg.servol.velocity,
                      moveArg.servol.acceleration,
                      servolTime,
                      moveArg.servol.lookaheadTime,
                      moveArg.servol.gain);
   auto t_stop = high_resolution_clock::now();
   auto t_duration = std::chrono::duration<double>(t_stop - t_start);
   if (t_duration.count() < servolTime)
   {
     std::this_thread::sleep_for(std::chrono::duration<double>( servolTime - t_duration.count()));
   }
 }

 //force control mode, the function is blocked
 void forceMode(void)
 {
   double cycleTime = 1.0 / CTRL_FREQ;
   auto t_start = high_resolution_clock::now();
   rtdeControl.forceMode(moveArg.force.task_frame,
                         moveArg.force.selection_vector,
                         moveArg.force.wrench,
                         moveArg.force.force_type,
                         moveArg.force.limitsSpeed);
   auto t_stop = high_resolution_clock::now();
   auto t_duration = std::chrono::duration<double>(t_stop - t_start);
   if (t_duration.count() < cycleTime)
   {
     std::this_thread::sleep_for(std::chrono::duration<double>(cycleTime - t_duration.count()));
   }
 }


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
    ros::AsyncSpinner AS(1);
    AS.start();

    urR.moveInit();
    printf("ur have moved to init pose\n");

//    usleep(200000);

    while(ros::ok())
    {
      urR.servolMove();
//      urR.forceMode();
    }
    delete &urR;
    return 0;
}
