#include <ros/ros.h>
#include <unistd.h>
#include <vector>
#include <deque>
#include <algorithm>
#include <math.h>
#include <Eigen/Dense>
#include <std_msgs/Float64.h>
#include <geometry_msgs/WrenchStamped.h>
#include <human_machine_coordination/RobotControl.h>
#include <human_machine_coordination/RobotData.h>
#include <data_processing.h>

double msgDelayTimeMs(ros::Time gettime){
  ros::Time timeNow = ros::Time::now();
  double timeInterval;
  timeInterval = ((timeNow.sec - gettime.sec)*1000000000 + timeNow.nsec -gettime.nsec)/1000000.0;
  return timeInterval;
}

class Process_FT_Data{
private:
//  FT_Data FTSensorErro;

//  bool FTNeedCalibration = true;
//  bool touchFTFlag = false;
public:
//  static const int FTDataBuffer = 50;
//  bool forceFlag = false;
//  char m_padding [7];
//  FT_Data ftData, ftDataMean;
//  std::deque<FT_Data> ftDeque;
  FT_Args args;

  //力传感器 kw_ft_data topic的消息回调函数
//  void ftDataCallback(const geometry_msgs::WrenchStamped &msg)
//  {
//    FT_Data sum;
//    //        printf("Fz= %2f Kg\n",msg.wrench.force.z);
//    //   printf("delay time ms = %2f\n",delayTimeMs(msg.header.stamp));
//    //     printf("fx = %2f\n",ftData.fx);
//    ftData.fx = static_cast<double>(msg.wrench.force.x) - FTSensorErro.fx;
//    ftData.fy = - static_cast<double>(msg.wrench.force.y) - FTSensorErro.fy;
//    ftData.fz = - static_cast<double>(msg.wrench.force.z) - FTSensorErro.fz;
//    ftData.tx = static_cast<double>(msg.wrench.torque.x) - FTSensorErro.tx;
//    ftData.ty = - static_cast<double>(msg.wrench.torque.y) - FTSensorErro.ty;
//    ftData.tz = - static_cast<double>(msg.wrench.torque.z) - FTSensorErro.tz;
//    if(ftDeque.size() > FTDataBuffer){
//      ftDeque.pop_front();
//      ftDeque.push_back(ftData);
//      for (unsigned long int i = ftDeque.size() - 8; i < ftDeque.size(); i++) {
//        sum.fx += ftDeque.at(i).fx;
//        sum.fy += ftDeque.at(i).fy;
//        sum.fz += ftDeque.at(i).fz;
//        sum.tx += ftDeque.at(i).tx;
//        sum.ty += ftDeque.at(i).ty;
//        sum.tz += ftDeque.at(i).tz;
//      }
//      ftDataMean.fx = sum.fx / 8.0;
//      ftDataMean.fy = sum.fy / 8.0;
//      ftDataMean.fz = sum.fz / 8.0;
//      ftDataMean.tx = sum.tx / 8.0;
//      ftDataMean.ty = sum.ty / 8.0;
//      ftDataMean.tz = sum.tz / 8.0;
//      //     printf(" %2f \n", ftDataMean.fx - ftData.fx);
//    }
//    else{
//      ftDeque.push_back(ftData);
//    }
//  }
  void ftDataCallback(const geometry_msgs::WrenchStamped &msg)
  {
    Eigen::Matrix<double,3,1> forceSum = Eigen::MatrixXd::Zero(3,1);
    args.force << msg.wrench.force.x, - msg.wrench.force.y, - msg.wrench.force.z;
    args.torque << msg.wrench.torque.x, - msg.wrench.torque.y, - msg.wrench.torque.z;
    args.force -= args.forceErro;
    args.torque -= args.torqueErro;

    if(args.forceDeque.size() >= args.FTDataBuffer){
      args.forceDeque.pop_front();
      args.forceDeque.push_back(args.force);

      for (unsigned long i = args.forceDeque.size() - 8; i < args.forceDeque.size(); i++) {
        forceSum += args.forceDeque.at(i);
      }
      args.meanForce = forceSum / 8.0;
      //      printf("delay time ms = %2f\n", msgDelayTimeMs(msg.header.stamp));
      //      printf("args.meanForce(0,0) =  %2f \n", args.meanForce(0,0));
    }
    else{
      args.forceDeque.push_back(args.force);
    }
  }

  //检测力传感器是否被触摸（会因为震动误判），和是否受力
//  bool ifTouch(void){

//    FT_Data sum, mean, accum, stdev;
//    double stdevSum;
//    const double maxStdevSum = 0.015;
//    const double maxFError = 0.2;
//    std::deque<FT_Data> tmpFTQ;
//    tmpFTQ = ftDeque;

//    //    struct timeval tpstart,tpend;
//    //    double timeuse;
//    //    gettimeofday(&tpstart, NULL);

//    std::for_each(std::begin(tmpFTQ), std::end(tmpFTQ), [&](const FT_Data data){
//      sum.fx += data.fx;
//      sum.fy += data.fy;
//      sum.fz += data.fz;
//      sum.tx += data.tx;
//      sum.ty += data.ty;
//      sum.tz += data.tz;
//    });
//    mean.fx = sum.fx / tmpFTQ.size();
//    mean.fy = sum.fy / tmpFTQ.size();
//    mean.fz = sum.fz / tmpFTQ.size();
//    mean.tx = sum.tx / tmpFTQ.size();
//    mean.ty = sum.ty / tmpFTQ.size();
//    mean.tz = sum.tz / tmpFTQ.size();
//    //    printf("Fz mean = %2f Kg\n",mean.fz);
//    //    printf("Fz = %2f Kg\n",ftData.fz);

//    std::for_each(std::begin(tmpFTQ), std::end(tmpFTQ), [&](const FT_Data data){
//      accum.fx += (data.fx - mean.fx) * (data.fx - mean.fx);
//      accum.fy += (data.fy - mean.fy) * (data.fy - mean.fy);
//      accum.fz += (data.fz - mean.fz) * (data.fz - mean.fz);
//      accum.tx += (data.tx - mean.tx) * (data.tx - mean.tx);
//      accum.ty += (data.ty - mean.ty) * (data.ty - mean.ty);
//      accum.tz += (data.tz - mean.tz) * (data.tz - mean.tz);
//    });
//    stdev.fx = sqrt(accum.fx/(tmpFTQ.size()-1));
//    stdev.fy = sqrt(accum.fy/(tmpFTQ.size()-1));
//    stdev.fz = sqrt(accum.fz/(tmpFTQ.size()-1));
//    stdev.tx = sqrt(accum.tx/(tmpFTQ.size()-1));
//    stdev.ty = sqrt(accum.ty/(tmpFTQ.size()-1));
//    stdev.tz = sqrt(accum.tz/(tmpFTQ.size()-1));
//    stdevSum = stdev.fx + stdev.fy + stdev.fz + stdev.tx + stdev.ty + stdev.tz;

//    //    gettimeofday(&tpend, NULL);
//    //    timeuse=1000000*(tpend.tv_sec-tpstart.tv_sec)+tpend.tv_usec-tpstart.tv_usec;//注意，秒的读数和微秒的读数都应计算在内
//    //    printf("used time:%fus\n",timeuse);

//    if(stdevSum > maxStdevSum){
//      touchFTFlag = true;
//      //      printf("stdevSum = %2f\n",stdevSum);
//      if(std::fabs(ftData.fx) > maxFError | std::fabs(ftData.fy) > maxFError | std::fabs(ftData.fz) > maxFError){
//        forceFlag = true;
//      }
//      else{
//        forceFlag = false;
//      }
//    }
//    else{
//      touchFTFlag = false;
//      forceFlag = false;
//    }
//    return touchFTFlag;
//  }
  bool ifTouch(std::deque<Eigen::Matrix<double,3,1>> tmpFTDeque){

    Eigen::Matrix<double,3,1> sum, mean, accum, stdev;
    sum = mean = accum = stdev = Eigen::MatrixXd::Zero(3,1);
    double stdevSum = 0 ;
    const double maxStdevSum = 0.015;
    const double maxFError = 0.2;

    //        struct timeval tpstart,tpend;
    //        double timeuse;
    //        gettimeofday(&tpstart, NULL);

    std::for_each(std::begin(tmpFTDeque), std::end(tmpFTDeque), [&](const Eigen::Matrix<double,3,1> data){
      sum += data;
    });
    mean = sum / tmpFTDeque.size();

    std::for_each(std::begin(tmpFTDeque), std::end(tmpFTDeque), [&](const Eigen::Matrix<double,3,1> data){
      accum += ((data - mean).array() * (data - mean).array()).matrix();
    });
    stdev = sqrt(accum.array()/(tmpFTDeque.size()-1)).matrix();
    stdevSum = stdev.rowwise().sum()(0,0);

    //        gettimeofday(&tpend, NULL);
    //        timeuse=1000000*(tpend.tv_sec-tpstart.tv_sec)+tpend.tv_usec-tpstart.tv_usec;//注意，秒的读数和微秒的读数都应计算在内
    //        printf("used time:%fus\n",timeuse);

    if(stdevSum > maxStdevSum){
      args.touchFTFlag = true;
      //      printf("stdevSum = %2f\n",stdevSum);
      if(args.meanForce.array().abs().maxCoeff() > maxFError){
        args.forceFlag = true;
      }
      else{
        args.forceFlag = false;
      }
    }
    else{
      args.touchFTFlag = false;
      args.forceFlag = false;
    }
    return args.touchFTFlag;
  }


  //校准力传感器
//  int setFTDateZero(void)
//  {
//    FT_Data sum, mean, accum, stdev;
//    if(FTNeedCalibration == true && touchFTFlag == false && ftDeque.size() >= FTDataBuffer){
//      FTNeedCalibration = false;
//    }
//    else {
//      printf("FT sensor data don't need to Calibrate. \n");
//      return 1;
//    }
//    std::deque<FT_Data> tmpFTQ;
//    tmpFTQ = ftDeque;
//    std::for_each(std::begin(tmpFTQ), std::end(tmpFTQ), [&](const FT_Data data){
//      sum.fx += data.fx;
//      sum.fy += data.fy;
//      sum.fz += data.fz;
//      sum.tx += data.tx;
//      sum.ty += data.ty;
//      sum.tz += data.tz;
//    });
//    mean.fx = sum.fx / tmpFTQ.size();
//    mean.fy = sum.fy / tmpFTQ.size();
//    mean.fz = sum.fz / tmpFTQ.size();
//    mean.tx = sum.tx / tmpFTQ.size();
//    mean.ty = sum.ty / tmpFTQ.size();
//    mean.tz = sum.tz / tmpFTQ.size();

//    //    printf("Fz mean = %2f Kg\n",mean.fz);
//    //    printf("Fz = %2f Kg\n",ftData.fz);

//    std::for_each(std::begin(tmpFTQ), std::end(tmpFTQ), [&](const FT_Data data){
//      accum.fx += (data.fx - mean.fx) * (data.fx - mean.fx);
//      accum.fy += (data.fy - mean.fy) * (data.fy - mean.fy);
//      accum.fz += (data.fz - mean.fz) * (data.fz - mean.fz);
//      accum.tx += (data.tx - mean.tx) * (data.tx - mean.tx);
//      accum.ty += (data.ty - mean.ty) * (data.ty - mean.ty);
//      accum.tz += (data.tz - mean.tz) * (data.tz - mean.tz);
//    });
//    stdev.fx = sqrt(accum.fx/(tmpFTQ.size()-1));
//    stdev.fy = sqrt(accum.fy/(tmpFTQ.size()-1));
//    stdev.fz = sqrt(accum.fz/(tmpFTQ.size()-1));
//    stdev.tx = sqrt(accum.tx/(tmpFTQ.size()-1));
//    stdev.ty = sqrt(accum.ty/(tmpFTQ.size()-1));
//    stdev.tz = sqrt(accum.tz/(tmpFTQ.size()-1));
//    printf("Fz stdev = %2f \n", stdev.fz);
//    //    printf("tz stdev = %2f Kg\n", stdev.fz);
//    if(stdev.fz < 0.005){
//      FTSensorErro.fx = mean.fx;
//      FTSensorErro.fy = mean.fy;
//      FTSensorErro.fz = mean.fz;
//      FTSensorErro.tx = mean.tx;
//      FTSensorErro.ty = mean.ty;
//      FTSensorErro.tz = mean.tz;
//      printf("FT sensor data have been Calibrated. \n");
//      return 0;
//    }
//    else{
//      printf("Stdev is too large to Calibrate FT sensor data. \n");
//      return 2;
//    }
//  }
  int setFTDateZero(std::deque<Eigen::Matrix<double,3,1>> tmpFTDeque)
  {
    Eigen::Matrix<double,3,1> sum, mean, accum, stdev;
    sum = mean = accum = stdev = Eigen::MatrixXd::Zero(3,1);
    double stdevSum = 0 ;
    const double maxStdevSum = 0.015;

    if(args.FTNeedCalibration == false){
      printf("FT sensor data don't need to Calibrate. \n");
      return 1;
    }
    std::for_each(std::begin(tmpFTDeque), std::end(tmpFTDeque), [&](const Eigen::Matrix<double,3,1> data){
      sum += data;
    });

    mean = sum / tmpFTDeque.size();

    std::for_each(std::begin(tmpFTDeque), std::end(tmpFTDeque), [&](const Eigen::Matrix<double,3,1> data){
      accum += ((data - mean).array() * (data - mean).array()).matrix();
    });
    stdev = sqrt(accum.array()/(tmpFTDeque.size()-1)).matrix();
    stdevSum = stdev.rowwise().sum()(0,0);

    if(stdevSum < maxStdevSum){
      args.forceErro = mean;
      printf("FT sensor data have been Calibrated. \n");
      args.FTNeedCalibration = false;
      return 0;
    }else{
      printf("Stdev is too large to Calibrate FT sensor data. \n");
      return -1;
    }
  }

};

class Process_Robot_Data
{
private:

public:
  Robot_Pose pose;
  Robot_Args args;

  //机器人消息的回调函数
//  void robotDataCallback(const human_machine_coordination::RobotData &msg)
//  {

//    pose.x = msg.actualTCPPose[0];
//    pose.y = msg.actualTCPPose[1];
//    pose.z = msg.actualTCPPose[2];
//    pose.Rx = msg.actualTCPPose[3];
//    pose.Ry = msg.actualTCPPose[4];
//    pose.Rz = msg.actualTCPPose[5];
//    //    printf("delay time ms = %2f\n",msgDelayTimeMs(msg.header.stamp));
//  }

  void robotDataCallback(const human_machine_coordination::RobotData &msg)
  {
    std::vector<double> tmp;
    tmp = msg.actualTCPPose;
    args.actualTCPPose = Eigen::Map<Eigen::Matrix<double,6,1>>(&tmp[0],tmp.size());
    tmp.clear();
    tmp = msg.actualTCPPose;
    args.targetTCPPose = Eigen::Map<Eigen::Matrix<double,6,1>>(&tmp[0],tmp.size());
    tmp.clear();
    tmp = msg.actualTCPSpeed;
    args.actualTCPSpeed = Eigen::Map<Eigen::Matrix<double,6,1>>(&tmp[0],tmp.size());
    tmp.clear();
    tmp = msg.targetTCPSpeed;
    args.targetTCPSpeed = Eigen::Map<Eigen::Matrix<double,6,1>>(&tmp[0],tmp.size());
    tmp.clear();
    tmp = msg.actualTCPForce;
    args.actualTCPForce = Eigen::Map<Eigen::Matrix<double,6,1>>(&tmp[0],tmp.size());
    tmp.clear();
    tmp = msg.actualToolAccel;
    args.actualToolAccel = Eigen::Map<Eigen::Matrix<double,3,1>>(&tmp[0],tmp.size());

    args.position = args.actualTCPPose.block(0,0,3,1);
    args.posture = args.actualTCPPose.block(3,0,3,1);
    //    printf("delay time ms = %2f\n", msgDelayTimeMs(msg.header.stamp));
    //    printf("actualTCPForce(0,0) = %2f\n", args.actualTCPForce(0,0));
  }
};

class Controller
{
private:
  Controller_Args args;
public:
//  Robot_Pose controlPose;
  Eigen::Matrix<double,6,1> controlPose;
  Eigen::Matrix<double,6,1> targetPose;
  Eigen::Matrix<double,6,1> targetVel;

  //输入：实际的三维力和实际的末端位置信息
  //输出：到机器人目标末端位置
//  void FP_Position(FT_Data FData, Robot_Pose Pose, bool forceFlag)
//  {
//    //     int len = sizeof(Rpose) / sizeof(Rpose.x);
//    //     Robot_Pose *pRPose = &Rpose;
//    //     Robot_Pose *pCPose = &controlPose;
//    controlPose = Pose;
//    double mapCoefficient = 0.015;
//    double maxF = 6.0;
//    if(FData.fx > maxF){
//      FData.fx = maxF;
//    }
//    if(FData.fy > maxF){
//      FData.fy = maxF;
//    }
//    if(FData.fz > maxF){
//      FData.fz = maxF;
//    }
//    if(forceFlag){
//      controlPose.x = Pose.x + FData.fx * mapCoefficient;
//      controlPose.y = Pose.y + FData.fy * mapCoefficient;
//      controlPose.z = Pose.z + FData.fz * mapCoefficient;
//    }
//    //    printf("FSize.fx: %f m\n", FSize.fx);
//  }
  void FP_Position(Eigen::Matrix<double,3,1> FData, Eigen::Matrix<double,6,1> pose, bool forceFlag)
  {
    controlPose = pose;
    if(!forceFlag) return;
    double mapCoefficient = 0.015;
    Eigen::Matrix<double,3,1> dtPosition = Eigen::MatrixXd::Zero(3,1);
    Eigen::Matrix<double,3,1> dtPosture = Eigen::MatrixXd::Zero(3,1);
    Eigen::Matrix<double,6,1> dtPose = Eigen::MatrixXd::Zero(6,1);

    Eigen::Matrix<double,Eigen::Dynamic,1> maxF;
    maxF.resize(FData.rows(),1);
    maxF.fill(6.0);
    for (int i = 0; i < FData.rows(); i++) {
      FData(i,0) = FData(i,0) < maxF(i,0) ? FData(i,0) : maxF(i,0);
    }
    dtPosition = FData * mapCoefficient;
    dtPose << dtPosition, dtPosture;
    //      printf(" dtPosition(0,0) = %2f \n", dtPosition(0,0));
    controlPose += dtPose;
//    printf("controlPoseM(0,0) %2f \n",controlPoseM(0,0));
    //    printf("FSize.fx: %f m\n", FSize.fx);
  }


  //输入：实际的三维力和实际的末端位置信息,虚拟势场
  //输出：到机器人目标末端位置
//  void FP_Position_VFFM(FT_Data FData, Robot_Pose Pose, bool forceFlag, Point VFFMData){
//    controlPose = Pose;
//    double mapCoefficient = 0.015;
//    double maxF = 6.0;
//    FT_Data dt;

//    if(forceFlag){
//      if(FData.fx > maxF){
//        FData.fx = maxF;
//      }
//      if(FData.fy > maxF){
//        FData.fy = maxF;
//      }
//      if(FData.fz > maxF){
//        FData.fz = maxF;
//      }
//      dt.fx = FData.fx * mapCoefficient * VFFMData.x;
//      dt.fy = FData.fy * mapCoefficient * VFFMData.y;
//      dt.fz = FData.fz * mapCoefficient * VFFMData.z;
//      controlPose.x = Pose.x + dt.fx;
//      controlPose.y = Pose.y + dt.fy;
//      controlPose.z = Pose.z + dt.fz;
//    }
//  }
  void FP_Position_VFFM(Eigen::Matrix<double,3,1> FData, Eigen::Matrix<double,6,1> pose, bool forceFlag, Eigen::Matrix<double,3,1> VFFMData){

    controlPose = pose;

    if(!forceFlag)return;

    Eigen::Matrix<double,3,1> dtPosition = Eigen::MatrixXd::Zero(3,1);
    Eigen::Matrix<double,3,1> dtPosture = Eigen::MatrixXd::Zero(3,1);
    Eigen::Matrix<double,6,1> dtPose = Eigen::MatrixXd::Zero(6,1);

    Eigen::Matrix<double,Eigen::Dynamic,1> coefficientF_P;
    coefficientF_P.resize(FData.rows(),1);
    coefficientF_P.fill(0.015);

    Eigen::Matrix<double,Eigen::Dynamic,1> maxF;
    maxF.resize(FData.rows(),1);
    maxF.fill(5.0);
    for (int i = 0; i < FData.rows(); i++) {
      FData(i,0) = FData(i,0) < maxF(i,0) ? FData(i,0) : maxF(i,0);
    }

    dtPosition = (FData.array() * VFFMData.array() * coefficientF_P.array()).matrix();
    dtPose << dtPosition, dtPosture;
    controlPose += dtPose;
  }

  //构造一个弹簧变阻尼系统
  //输入：实际的三维力和计算得到的上一位置末端位置，速度信息, 虚拟势场
  //输出：到机器人目标末端位置
  void PVF_Position(Eigen::Matrix<double,6,1> pose, Eigen::Matrix<double,6,1> vel, Eigen::Matrix<double,3,1> FData, bool touchFlag)
  {
    controlPose = pose;
    double targetK2;
    if(!touchFlag) return;
    Eigen::Matrix<double,2,2> A;
    Eigen::Matrix<double,2,1> B;

    Eigen::Matrix<double,2,3> state;
    Eigen::Matrix<double,2,3> dtSate = Eigen::MatrixXd::Zero(2,3);
    Eigen::Matrix<double,3,1> dtPosition = Eigen::MatrixXd::Zero(3,1);
    Eigen::Matrix<double,3,1> dtPosture = Eigen::MatrixXd::Zero(3,1);
    Eigen::Matrix<double,6,1> dtPose = Eigen::MatrixXd::Zero(6,1);
    Eigen::Matrix<double,6,1> dtVel = Eigen::MatrixXd::Zero(6,1);

    Eigen::Matrix<double,Eigen::Dynamic,1> maxF;
    maxF.resize(FData.rows(),1);
    maxF.fill(5.0);

    double vectorFLen = sqrt(pow(FData.array(), 2).sum());

    targetK2 = args.K2 - 1.0 / pow(vectorFLen, 2.8613);
    if(targetK2 < -100){
      targetK2 = -100;
    }

    A << 0, 1, args.K1, targetK2;
    B << 0, args.K3;

    state << pose.block(0,0,3,1).transpose(),vel.block(0,0,3,1).transpose();

    for (int i = 0; i < FData.rows(); i++) {
      FData(i,0) = FData(i,0) < maxF(i,0) ? FData(i,0) : maxF(i,0);
      if(vectorFLen < args.K4){
        FData(i,0) = 0;
      }else{
        FData(i,0) = FData(i,0) > 0.0 ?
              FData(i,0) - args.K4 * fabs(FData(i,0)) / vectorFLen :
              FData(i,0) + args.K4 * fabs(FData(i,0)) / vectorFLen;
      }
    }
    dtSate = A * state + B * FData.transpose();
    //        printf("FData(0,0) %2f \n",FData(0,0));

    dtPosition = (dtSate.block(1,0,1,3).transpose() * 1.0 / CTRL_FREQ + vel.block(0,0,3,1) * 2.0) / 2.0 * 1.0 / CTRL_FREQ;
    dtPose << dtPosition, dtPosture;

    controlPose += dtPose;

    targetPose = controlPose;
    dtVel << dtSate.block(1,0,1,3).transpose() / CTRL_FREQ, 0 ,0 ,0 ;
    targetVel += dtVel;
    //printf("FData(0,0) %2f \n",FData(0,0));
    //printf("targetVel(0,0) %2f \n",targetVel(0,0));
    //printf("dtSate(1,0) %2f \n",dtSate(1,0));
    //printf("dtPose(0,0) %2f \n",dtPose(0,0));
  }
};

class Robot_Move{
private:

public:
//  void sendPoseMsg(ros::Publisher  urConParPub, Robot_Pose pose){
//    human_machine_coordination::RobotControl msg;
//    msg.pose.x = pose.x;
//    msg.pose.y = pose.y;
//    msg.pose.z = pose.z;
//    msg.pose.Rx = pose.Rx;
//    msg.pose.Ry = pose.Ry;
//    msg.pose.Rz = pose.Rz;
//    msg.header.stamp = ros::Time::now();
//    urConParPub.publish(msg);
//  }

  void sendPoseMsg(ros::Publisher  urConParPub, Eigen::Matrix<double,6,1> pose){
    human_machine_coordination::RobotControl msg;
    msg.pose.x = pose(0,0);
    msg.pose.y = pose(1,0);
    msg.pose.z = pose(2,0);
    msg.pose.Rx = pose(3,0);
    msg.pose.Ry = pose(4,0);
    msg.pose.Rz = pose(5,0);
    msg.header.stamp = ros::Time::now();
    urConParPub.publish(msg);
  }
};

class VFFM{
private:
//计算点到平面的距离，点在平面的哪个方向（法线方向为正），平面单位法矢量
//  void relationPointPlane(Plane *plane, Point point)
//  {
//    double tmpDirection, tmpABC;
//    tmpDirection = plane->fun.A * (point.x - plane->fun.x0) +
//        plane->fun.B * (point.y - plane->fun.y0) +
//        plane->fun.C * (point.z - plane->fun.z0);
//    if(tmpDirection > 0.0){
//      plane->direction = 1.0;
//    }else{
//      plane->direction = -1.0;
//    }
//    tmpABC = sqrt(pow(plane->fun.A,2) + pow(plane->fun.B,2) + pow(plane->fun.C,2));
//    plane->distance = fabs(tmpDirection) / tmpABC;
//    plane->unitNormalVector.x = plane->fun.A / tmpABC;
//    plane->unitNormalVector.y = plane->fun.B / tmpABC;
//    plane->unitNormalVector.z = plane->fun.C / tmpABC;
//  }
  void relationPointPlane(Plane *plane, Eigen::Matrix<double,3,1> position)
  {
    double tmpDirection, tmpABC;
    tmpDirection = (plane->funM.normalVector.array() *
                    (position - plane->funM.P0).array()).sum();

    if(tmpDirection > 0.0){
      plane->direction = 1.0;
    }else{
      plane->direction = -1.0;
    }
    tmpABC = sqrt(pow(plane->funM.normalVector.array(), 2).sum());
    plane->distance = fabs(tmpDirection) / tmpABC;
    plane->unitNormalVectorM = plane->funM.normalVector / tmpABC;
//    printf("plane->distance = %2f \n", plane->distance);
  }

public:
  Point endPose;
  FT_Data endFT;
  std::deque<Plane> planes;
  //计算一个平面的虚拟力场
//  double PlaneVF(Plane *plane, Point position)
//  {
//    double tmpVF;
//    relationPointPlane(plane, position);

//    if(plane->distance > plane->rho)
//    {
//      tmpVF = 1.0;
//    }else{
//      switch (plane->arithmetic)
//      {
//      case Trigonometric:
//        tmpVF = -0.5 * cos((1 / plane->rho) * M_PI * plane->distance) + 0.5;
//        break;
//      case HalfTrigonometric:
//        tmpVF = sin((0.5 / plane->rho) * M_PI * plane->distance);
//        break;
//      case Power:
//        tmpVF = pow(plane->distance, plane->lambda) / pow(plane->rho, plane->lambda);
//        break;
//      default:
//        tmpVF = 0.0;
//        break;
//      }
//      if(tmpVF > 1.0){
//        tmpVF = 1.0;
//      }
//    }
//    if(plane->direction > 0.0){
//      tmpVF = -tmpVF;
//    }
//    plane->virtualForce = tmpVF;
//    return plane->virtualForce;
//  }
  double PlaneVF(Plane *plane, Eigen::Matrix<double,3,1> position)
  {
    double tmpVF;
    relationPointPlane(plane, position);
    if(plane->distance > plane->rho)
    {
      tmpVF = 1.0;
    }else{
      switch (plane->arithmetic)
      {
      case Trigonometric:
        tmpVF = -0.5 * cos((1 / plane->rho) * M_PI * plane->distance) + 0.5;
        break;
      case HalfTrigonometric:
        tmpVF = sin((0.5 / plane->rho) * M_PI * plane->distance);
        break;
      case Power:
        tmpVF = pow(plane->distance, plane->lambda) / pow(plane->rho, plane->lambda);
        break;
      default:
        tmpVF = 0.0;
        break;
      }
      if(tmpVF > 1.0){
        tmpVF = 1.0;
      }
    }
    if(plane->direction > 0.0){
      tmpVF = -tmpVF;
    }
    plane->virtualForce = tmpVF;
//    printf("plane->virtualForce = %2f \n", plane->virtualForce);
    return plane->virtualForce;
  }

  //计算所有平面的虚拟力场的矢量和
//  Point PlanesVFFWorld(std::deque<Plane> planes, Point point, FT_Data FTData)
//  {
//    Point VFWorld, maxVF;
//    double tmpInnerProduct;
//    std::for_each(std::begin(planes), std::end(planes), [&](Plane plane){
//      PlaneVF(&plane, point);

//      //The inner product of real forces and virtual forces
//      tmpInnerProduct = plane.unitNormalVector.x * FTData.fx +
//          plane.unitNormalVector.y * FTData.fy +
//          plane.unitNormalVector.z * FTData.fz;
//      if(fabs(tmpInnerProduct) > 0.01 && tmpInnerProduct < 0.0){
//        return;
//      }
//      //      printf("tmpInnerProduct = %2f \n", tmpInnerProduct);
//      VFWorld.x += - plane.virtualForce * plane.unitNormalVector.x;
//      VFWorld.y += - plane.virtualForce * plane.unitNormalVector.y;
//      VFWorld.z += - plane.virtualForce * plane.unitNormalVector.z;
//      maxVF.x += - 1.0 * plane.unitNormalVector.x;
//      maxVF.y += - 1.0 * plane.unitNormalVector.y;
//      maxVF.z += - 1.0 * plane.unitNormalVector.z;
//      //      printf("plane.virtualForce = %2f \n", plane.virtualForce);
//      //      printf("plane.fun.z = %2f \n", plane.fun.C);
//      //      printf("plane.unitNormalVector.z = %2f \n", plane.unitNormalVector.z);
//    });
//    if(maxVF.x != 0.0){
//      VFWorld.x = VFWorld.x / maxVF.x;
//    }else{
//      VFWorld.x = 1.0;
//    }
//    if(maxVF.y != 0.0){
//      VFWorld.y = VFWorld.y / maxVF.y;
//    }else{
//      VFWorld.y = 1.0;
//    }
//    if(maxVF.z != 0.0){
//      VFWorld.z = VFWorld.z / maxVF.z;
//    }else{
//      VFWorld.z = 1.0;
//    }
//    return VFWorld;
//  }
  Eigen::Matrix<double,3,1> PlanesVFFWorld(std::deque<Plane> planes,    Eigen::Matrix<double,3,1> position, Eigen::Matrix<double,3,1> FData)
  {
    Eigen::Matrix<double,3,1> VFWorld = Eigen::MatrixXd::Zero(3,1);
    Eigen::Matrix<double,3,1> maxVF = Eigen::MatrixXd::Zero(3,1);
    double tmpInnerProduct;
    std::for_each(std::begin(planes), std::end(planes), [&](Plane plane){
      PlaneVF(&plane, position);

      //The inner product of real forces and virtual forces
      tmpInnerProduct = (plane.unitNormalVectorM.array() * FData.array()).sum();
      if(fabs(tmpInnerProduct) > 0.01 && tmpInnerProduct < 0.0){
//        printf("return \n");
        return;
      }
      //      printf("tmpInnerProduct = %2f \n", tmpInnerProduct);
      VFWorld += - plane.virtualForce * plane.unitNormalVectorM;
      maxVF += - 1.0 * plane.unitNormalVectorM;
    });

    if(maxVF(0,0) != 0.0){
      VFWorld(0,0) = VFWorld(0,0) / maxVF(0,0);
    }else{
      VFWorld(0,0) = 1.0;
    }
    if(maxVF(1,0) != 0.0){
      VFWorld(1,0) = VFWorld(1,0) / maxVF(1,0);
    }else{
      VFWorld(1,0) = 1.0;
    }
    if(maxVF(2,0) != 0.0){
      VFWorld(2,0) = VFWorld(2,0) / maxVF(2,0);
    }else{
      VFWorld(2,0) = 1.0;
    }
//    printf("VFWorld(0,0) = %2f \n", VFWorld(0,0));
    return VFWorld;
  }
};


int main(int argc, char **argv)
{
  Process_FT_Data processFTData;
  Process_Robot_Data processURData;
  Controller controller;
  Robot_Move robotMove;
  VFFM planeVFFM;

  ros::init(argc, argv, "data_processing");
  ros::NodeHandle n;
  ros::Subscriber FTDataSub = n.subscribe("kw_ft_data", 100, &Process_FT_Data::ftDataCallback, &processFTData);
  ros::Subscriber URDataSub = n.subscribe("hmc_ur_data", 100, &Process_Robot_Data::robotDataCallback, &processURData);
  ros::Publisher  urConParPub = n.advertise<human_machine_coordination::RobotControl>("hmc_ur_control", 100);
  ros::Rate loop_rate(CTRL_FREQ);
  ros::AsyncSpinner AS(1);
  AS.start();

  Eigen::Matrix<double,3,1> VFWorld = Eigen::MatrixXd::Zero(3,1);
  Plane x400;
  x400.funM.normalVector << 1, 0, 0;//A,B,C
  x400.funM.P0 << 0.4, 0, 0;//x0,y0,z0
  x400.lambda = 0.3;
  x400.arithmetic = Trigonometric;
  planeVFFM.planes.push_back(x400);
  Plane yN600;
  yN600.funM.normalVector << 0, -1, 0;//A,B,C
  yN600.funM.P0 << 0, -0.5, 0;//x0,y0,z0
  yN600.lambda = 0.3;
  yN600.arithmetic = Trigonometric;
  planeVFFM.planes.push_back(yN600);
  Plane z300;
  z300.lambda = 0.3;
  z300.arithmetic = Trigonometric;
  z300.funM.normalVector << 0, 0, -1;//A,B,C
  z300.funM.P0 << 0, 0, 0.2;//x0,y0,z0
  planeVFFM.planes.push_back(z300);

  controller.targetPose << 0.3, -0.45, 0.3, 0.0, 3.14, 0.0;
  controller.targetVel << 0.0,0.0,0.0,0.0,0.0,0.0;

  //等待力传感器数据队列存满
  while(processFTData.args.forceDeque.size() < processFTData.args.FTDataBuffer){
    loop_rate.sleep();
  }

  //等待力传感器完成校准
  while(processFTData.args.FTNeedCalibration == true){
    processFTData.setFTDateZero(processFTData.args.forceDeque);
  }

  //
  usleep(5000000);

  while(ros::ok())
  {
    processFTData.ifTouch(processFTData.args.forceDeque);

//    planeVFFM.endPose.x = processURData.pose.x;
//    planeVFFM.endPose.y = processURData.pose.y;
//    planeVFFM.endPose.z = processURData.pose.z;
//    planeVFFM.endFT = processFTData.ftDataMean;

    VFWorld = planeVFFM.PlanesVFFWorld(planeVFFM.planes, processURData.args.position, processFTData.args.meanForce);

//    controller.FP_Position(processFTData.args.meanForce, processURData.args.actualTCPPose, processFTData.args.forceFlag);

    controller.FP_Position_VFFM(processFTData.args.meanForce, processURData.args.actualTCPPose, processFTData.args.forceFlag, VFWorld);

//    controller.PVF_Position(controller.targetPose, controller.targetVel, processFTData.args.meanForce, processFTData.args.touchFTFlag);

    robotMove.sendPoseMsg(urConParPub, controller.controlPose);
    loop_rate.sleep();
  }
  return 0;
}




//不用eigen库的main函数更新代码
//要更改前面的消息回调函数
//  processFTData.setFTDateZero();
//  while(ros::ok())
//  {
//    processFTData.ifTouch();
//    planeVFFM.endPose.x = processURData.pose.x;
//    planeVFFM.endPose.y = processURData.pose.y;
//    planeVFFM.endPose.z = processURData.pose.z;
//    planeVFFM.endFT = processFTData.ftDataMean;
//    VFWorld = planeVFFM.PlanesVFFWorld(planeVFFM.planes, planeVFFM.endPose, planeVFFM.endFT);
//    //    data = planeVFFM.planeVFFM_InverseProportion().front();
//    //    printf(" %2f \n", planeVFFM.planeVFFM_InverseProportion().front());

//    //    controller.FP_Position(processFTData.ftDataMean, processURData.pose, processFTData.forceFlag);
//    controller.FP_Position_VFFM(processFTData.ftDataMean, processURData.pose, processFTData.forceFlag, VFWorld);
//    robotMove.sendPoseMsg(urConParPub, controller.controlPose);
//    //    printf(" %2f \n", processURData.pose.x - mapAlgorithm.controlPose.x);
//    loop_rate.sleep();
//  }
