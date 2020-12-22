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

class Processing_FT_Data{
private:
  FT_Data FTSensorErro, maxFT;
  bool FTNeedCalibration = true;
  bool touchFTFlag = false;
public:
  static const int FTDataBuffer = 50;
  bool forceFlag = false;
  char m_padding [5];
  FT_Data ftData, ftDataMean;
  std::deque<FT_Data> ftDeque;

  //力传感器 kw_ft_data topic的消息回调函数
  void ftDataCallback(const geometry_msgs::WrenchStamped &msg)
  {
    FT_Data sum;
//        printf("Fz= %2f Kg\n",msg.wrench.force.z);
    //   printf("delay time ms = %2f\n",delayTimeMs(msg.header.stamp));
//     printf("fx = %2f\n",ftData.fx);
    ftData.fx = static_cast<double>(msg.wrench.force.x) - FTSensorErro.fx;
    ftData.fy = - static_cast<double>(msg.wrench.force.y) - FTSensorErro.fy;
    ftData.fz = - static_cast<double>(msg.wrench.force.z) - FTSensorErro.fz;
    ftData.tx = static_cast<double>(msg.wrench.torque.x) - FTSensorErro.tx;
    ftData.ty = - static_cast<double>(msg.wrench.torque.y) - FTSensorErro.ty;
    ftData.tz = - static_cast<double>(msg.wrench.torque.z) - FTSensorErro.tz;
    if(ftDeque.size() > FTDataBuffer){
     ftDeque.pop_front();
     ftDeque.push_back(ftData);
     for (unsigned long int i = ftDeque.size() - 8; i < ftDeque.size(); i++) {
       sum.fx += ftDeque.at(i).fx;
       sum.fy += ftDeque.at(i).fy;
       sum.fz += ftDeque.at(i).fz;
       sum.tx += ftDeque.at(i).tx;
       sum.ty += ftDeque.at(i).ty;
       sum.tz += ftDeque.at(i).tz;
     }
     ftDataMean.fx = sum.fx / 8.0;
     ftDataMean.fy = sum.fy / 8.0;
     ftDataMean.fz = sum.fz / 8.0;
     ftDataMean.tx = sum.tx / 8.0;
     ftDataMean.ty = sum.ty / 8.0;
     ftDataMean.tz = sum.tz / 8.0;
//     printf(" %2f \n", ftDataMean.fx - ftData.fx);
    }
    else{
      ftDeque.push_back(ftData);
    }
  }

  //  Detect if the FT sensor has been touched
  bool ifTouch(void){

    FT_Data sum, mean, accum, stdev;
    double stdevSum;
    const double maxStdevSum = 0.015;
    const double maxFError = 0.2;
    std::deque<FT_Data>::iterator it;
    std::deque<FT_Data> tmpFTQ;
    tmpFTQ = ftDeque;

//    struct timeval tpstart,tpend;
//    double timeuse;
//    gettimeofday(&tpstart, NULL);

    std::for_each(std::begin(tmpFTQ), std::end(tmpFTQ), [&](const FT_Data data){
      sum.fx += data.fx;
      sum.fy += data.fy;
      sum.fz += data.fz;
      sum.tx += data.tx;
      sum.ty += data.ty;
      sum.tz += data.tz;
    });
    mean.fx = sum.fx / tmpFTQ.size();
    mean.fy = sum.fy / tmpFTQ.size();
    mean.fz = sum.fz / tmpFTQ.size();
    mean.tx = sum.tx / tmpFTQ.size();
    mean.ty = sum.ty / tmpFTQ.size();
    mean.tz = sum.tz / tmpFTQ.size();
    //    printf("Fz mean = %2f Kg\n",mean.fz);
    //    printf("Fz = %2f Kg\n",ftData.fz);

    std::for_each(std::begin(tmpFTQ), std::end(tmpFTQ), [&](const FT_Data data){
      accum.fx += (data.fx - mean.fx) * (data.fx - mean.fx);
      accum.fy += (data.fy - mean.fy) * (data.fy - mean.fy);
      accum.fz += (data.fz - mean.fz) * (data.fz - mean.fz);
      accum.tx += (data.tx - mean.tx) * (data.tx - mean.tx);
      accum.ty += (data.ty - mean.ty) * (data.ty - mean.ty);
      accum.tz += (data.tz - mean.tz) * (data.tz - mean.tz);
    });
    stdev.fx = sqrt(accum.fx/(tmpFTQ.size()-1));
    stdev.fy = sqrt(accum.fy/(tmpFTQ.size()-1));
    stdev.fz = sqrt(accum.fz/(tmpFTQ.size()-1));
    stdev.tx = sqrt(accum.tx/(tmpFTQ.size()-1));
    stdev.ty = sqrt(accum.ty/(tmpFTQ.size()-1));
    stdev.tz = sqrt(accum.tz/(tmpFTQ.size()-1));
    stdevSum = stdev.fx + stdev.fy + stdev.fz + stdev.tx + stdev.ty + stdev.tz;

//    gettimeofday(&tpend, NULL);
//    timeuse=1000000*(tpend.tv_sec-tpstart.tv_sec)+tpend.tv_usec-tpstart.tv_usec;//注意，秒的读数和微秒的读数都应计算在内
//    printf("used time:%fus\n",timeuse);

    if(stdevSum > maxStdevSum){
      touchFTFlag = true;
//      printf("stdevSum = %2f\n",stdevSum);
      if(std::fabs(ftData.fx) > maxFError | std::fabs(ftData.fy) > maxFError | std::fabs(ftData.fz) > maxFError){
        forceFlag = true;
      }
      else{
        forceFlag = false;
      }
    }
    else{
      touchFTFlag = false;
      forceFlag = false;
    }
    return touchFTFlag;
  }


  //Calibration of FT sensor data
  int setFTDateZero(void)
  {
    std::deque<FT_Data>::iterator it;
    FT_Data sum, mean, accum, stdev;
    if(FTNeedCalibration == true && touchFTFlag == false && ftDeque.size() >= FTDataBuffer){
      FTNeedCalibration = false;
    }
    else {
      printf("FT sensor data don't need to Calibrate. \n");
      return 1;
    }
    std::deque<FT_Data> tmpFTQ;
    tmpFTQ = ftDeque;
    std::for_each(std::begin(tmpFTQ), std::end(tmpFTQ), [&](const FT_Data data){
      sum.fx += data.fx;
      sum.fy += data.fy;
      sum.fz += data.fz;
      sum.tx += data.tx;
      sum.ty += data.ty;
      sum.tz += data.tz;
    });
    mean.fx = sum.fx / tmpFTQ.size();
    mean.fy = sum.fy / tmpFTQ.size();
    mean.fz = sum.fz / tmpFTQ.size();
    mean.tx = sum.tx / tmpFTQ.size();
    mean.ty = sum.ty / tmpFTQ.size();
    mean.tz = sum.tz / tmpFTQ.size();

    //    printf("Fz mean = %2f Kg\n",mean.fz);
    //    printf("Fz = %2f Kg\n",ftData.fz);

    std::for_each(std::begin(tmpFTQ), std::end(tmpFTQ), [&](const FT_Data data){
      accum.fx += (data.fx - mean.fx) * (data.fx - mean.fx);
      accum.fy += (data.fy - mean.fy) * (data.fy - mean.fy);
      accum.fz += (data.fz - mean.fz) * (data.fz - mean.fz);
      accum.tx += (data.tx - mean.tx) * (data.tx - mean.tx);
      accum.ty += (data.ty - mean.ty) * (data.ty - mean.ty);
      accum.tz += (data.tz - mean.tz) * (data.tz - mean.tz);
    });
    stdev.fx = sqrt(accum.fx/(tmpFTQ.size()-1));
    stdev.fy = sqrt(accum.fy/(tmpFTQ.size()-1));
    stdev.fz = sqrt(accum.fz/(tmpFTQ.size()-1));
    stdev.tx = sqrt(accum.tx/(tmpFTQ.size()-1));
    stdev.ty = sqrt(accum.ty/(tmpFTQ.size()-1));
    stdev.tz = sqrt(accum.tz/(tmpFTQ.size()-1));
    printf("Fz stdev = %2f \n", stdev.fz);
    //    printf("tz stdev = %2f Kg\n", stdev.fz);
    if(stdev.fz < 0.005){
      FTSensorErro.fx = mean.fx;
      FTSensorErro.fy = mean.fy;
      FTSensorErro.fz = mean.fz;
      FTSensorErro.tx = mean.tx;
      FTSensorErro.ty = mean.ty;
      FTSensorErro.tz = mean.tz;
      printf("FT sensor data have been Calibrated. \n");
      return 0;
    }
    else{
      printf("Stdev is too large to Calibrate FT sensor data. \n");
      return 2;
    }
  }
};

class Processing_Robot_Data{
private:

public:
  Robot_Pose pose, poseDifferce;
  //callback of robot data
  void robotDataCallback(const human_machine_coordination::RobotData &msg)
  {
    poseDifferce.x = msg.pose.x - pose.x;
    poseDifferce.y = msg.pose.x - pose.y;
    poseDifferce.z = msg.pose.x - pose.z;
    poseDifferce.Rx = msg.pose.x - pose.Rx;
    poseDifferce.Ry = msg.pose.x - pose.Ry;
    poseDifferce.Rz = msg.pose.x - pose.Rz;

    pose.x = msg.pose.x;
    pose.y = msg.pose.y;
    pose.z = msg.pose.z;
    pose.Rx = msg.pose.Rx;
    pose.Ry = msg.pose.Ry;
    pose.Rz = msg.pose.Rz;
//    printf("delay time ms = %2f\n",delayTimeMs(msg.header.stamp));
  }
};

class Map_FT_Robot{
private:

public:
  Robot_Pose controlPose;
  void mapFPose(FT_Data FData, Robot_Pose RPose, bool forceFlag)
  {
    //     int len = sizeof(Rpose) / sizeof(Rpose.x);
    //     Robot_Pose *pRPose = &Rpose;
    //     Robot_Pose *pCPose = &controlPose;
    controlPose = RPose;
    double mapCoefficient = 0.015;
    double maxF = 6.0;
    if(FData.fx > maxF){
      FData.fx = maxF;
    }
    if(FData.fy > maxF){
      FData.fy = maxF;
    }
    if(FData.fz > maxF){
      FData.fz = maxF;
    }
    if(forceFlag){
      controlPose.x = RPose.x + FData.fx * mapCoefficient;
      controlPose.y = RPose.y + FData.fy * mapCoefficient;
      controlPose.z = RPose.z + FData.fz * mapCoefficient;
    }
//    printf("FSize.fx: %f m\n", FSize.fx);
  }
  void mapFPoseVFFM(FT_Data FData, Robot_Pose RPose, bool forceFlag, Point VFFMData){
    controlPose = RPose;
    double mapCoefficient = 0.015;
    double maxF = 6.0;
    FT_Data dt;

    if(forceFlag){
      if(FData.fx > maxF){
        FData.fx = maxF;
      }
      if(FData.fy > maxF){
        FData.fy = maxF;
      }
      if(FData.fz > maxF){
        FData.fz = maxF;
      }
        dt.fx = FData.fx * mapCoefficient * VFFMData.x;
        dt.fy = FData.fy * mapCoefficient * VFFMData.y;
        dt.fz = FData.fz * mapCoefficient * VFFMData.z;
      controlPose.x = RPose.x + dt.fx;
      controlPose.y = RPose.y + dt.fy;
      controlPose.z = RPose.z + dt.fz;
    }
  }
};

class Contral_Robot{
private:

public:
  void sendRobotcontral(ros::Publisher  urConParPub, Robot_Pose pose){
    human_machine_coordination::RobotControl msg;
    msg.pose.x = pose.x;
    msg.pose.y = pose.y;
    msg.pose.z = pose.z;
    msg.pose.Rx = pose.Rx;
    msg.pose.Ry = pose.Ry;
    msg.pose.Rz = pose.Rz;
    msg.header.stamp = ros::Time::now();
    urConParPub.publish(msg);
  }
};

class VFFM{
private:
  //Calculate the distance from the point to the surface
  //Calculate which direction the point is in the plane
  void relationPointPlane(Plane *plane, Point point)
  {
    double tmpDirection, tmpABC;
    tmpDirection = plane->fun.A * (point.x - plane->fun.x0) +
        plane->fun.B * (point.y - plane->fun.y0) +
        plane->fun.C * (point.z - plane->fun.z0);
    if(tmpDirection > 0.0){
      plane->direction = 1.0;
    }else{
      plane->direction = -1.0;
    }
   tmpABC = sqrt(pow(plane->fun.A,2) + pow(plane->fun.B,2) + pow(plane->fun.C,2));
   plane->distance = fabs(tmpDirection) / tmpABC;
   plane->unitNormalVector.x = plane->fun.A / tmpABC;
   plane->unitNormalVector.y = plane->fun.B / tmpABC;
   plane->unitNormalVector.z = plane->fun.C / tmpABC;
  }

public:
  Point endPose;
  FT_Data endFT;
  std::deque<Plane> planes;
  //Calculate the value of the virtual potential field force by 1/x
  double PlaneVF(Plane *plane, Point point)
  {
    double tmpVF;
    relationPointPlane(plane, point);

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
    return plane->virtualForce;
  }

  //Integrate the forces of each target
  Point PlanesVFFWorld(std::deque<Plane> planes, Point point, FT_Data FTData)
  {
    Point VFWorld, maxVF;
    double tmpInnerProduct;
    std::for_each(std::begin(planes), std::end(planes), [&](Plane plane){
      PlaneVF(&plane, point);

      //The inner product of real forces and virtual forces
      tmpInnerProduct = plane.unitNormalVector.x * FTData.fx +
          plane.unitNormalVector.y * FTData.fy +
          plane.unitNormalVector.z * FTData.fz;
      if(fabs(tmpInnerProduct) > 0.01 && tmpInnerProduct < 0.0){
        return;
      }
//      printf("tmpInnerProduct = %2f \n", tmpInnerProduct);
      VFWorld.x += - plane.virtualForce * plane.unitNormalVector.x;
      VFWorld.y += - plane.virtualForce * plane.unitNormalVector.y;
      VFWorld.z += - plane.virtualForce * plane.unitNormalVector.z;
      maxVF.x += - 1.0 * plane.unitNormalVector.x;
      maxVF.y += - 1.0 * plane.unitNormalVector.y;
      maxVF.z += - 1.0 * plane.unitNormalVector.z;
//      printf("plane.virtualForce = %2f \n", plane.virtualForce);
//      printf("plane.fun.z = %2f \n", plane.fun.C);
//      printf("plane.unitNormalVector.z = %2f \n", plane.unitNormalVector.z);
    });
    if(maxVF.x != 0.0){
      VFWorld.x = VFWorld.x / maxVF.x;
    }else{
      VFWorld.x = 1.0;
    }
    if(maxVF.y != 0.0){
       VFWorld.y = VFWorld.y / maxVF.y;
    }else{
      VFWorld.y = 1.0;
    }
    if(maxVF.z != 0.0){
       VFWorld.z = VFWorld.z / maxVF.z;
    }else{
      VFWorld.z = 1.0;
    }
    return VFWorld;
  }
};


int main(int argc, char **argv)
{
  Processing_FT_Data processFTData;
  Processing_Robot_Data processURData;
  Map_FT_Robot mapAlgorithm;
  Contral_Robot controlRobot;
  VFFM planeVFFM;
  ros::init(argc, argv, "data_processing");
  ros::NodeHandle n;

  ros::Subscriber FTDataSub = n.subscribe("kw_ft_data", 100, &Processing_FT_Data::ftDataCallback, &processFTData);
  ros::Subscriber URDataSub = n.subscribe("hmc_ur_data", 100, &Processing_Robot_Data::robotDataCallback, &processURData);
  ros::Publisher  urConParPub = n.advertise<human_machine_coordination::RobotControl>("hmc_ur_control", 100);

  ros::AsyncSpinner AS(1);
  AS.start();

  Point VFWorld;


  Plane x400;
  x400.fun.A = 1;
  x400.fun.B = 0;
  x400.fun.C = 0;
  x400.fun.x0 = 0.4;
  x400.fun.y0= 0;
  x400.fun.z0 = 0; //0.146;
  x400.lambda = 5;
  x400.arithmetic = Trigonometric;
  planeVFFM.planes.push_back(x400);

  Plane yN600;
  yN600.fun.A = 0;
  yN600.fun.B = -1;
  yN600.fun.C = 0;
  yN600.fun.x0 = 0;
  yN600.fun.y0= -0.6;
  yN600.fun.z0 = 0; //0.146;
  yN600.lambda = 0.3;
  yN600.arithmetic = Trigonometric;
  planeVFFM.planes.push_back(yN600);

  Plane z300;
  z300.fun.A = 0;
  z300.fun.B = 0;
  z300.fun.C = -1;
  z300.fun.x0 = 0;
  z300.fun.y0= 0;
  z300.fun.z0 = 0.3; //0.146;
  z300.lambda = 0.3;
  z300.arithmetic = Trigonometric;
  planeVFFM.planes.push_back(z300);

  ros::Rate loop_rate(128);

  while(processFTData.ftDeque.size() < processFTData.FTDataBuffer)
  {
    //    ros::spinOnce();
    loop_rate.sleep();
  }

  //  usleep(200000);

  processFTData.setFTDateZero();
  while(ros::ok())
  {
    processFTData.ifTouch();

    planeVFFM.endPose.x = processURData.pose.x;
    planeVFFM.endPose.y = processURData.pose.y;
    planeVFFM.endPose.z = processURData.pose.z;
    planeVFFM.endFT = processFTData.ftDataMean;
    VFWorld = planeVFFM.PlanesVFFWorld(planeVFFM.planes, planeVFFM.endPose, planeVFFM.endFT);
//    data = planeVFFM.planeVFFM_InverseProportion().front();
//    printf(" %2f \n", planeVFFM.planeVFFM_InverseProportion().front());


//    mapAlgorithm.mapFPose(processFTData.ftDataMean, processURData.pose, processFTData.forceFlag);
    mapAlgorithm.mapFPoseVFFM(processFTData.ftDataMean, processURData.pose, processFTData.forceFlag, VFWorld);
    controlRobot.sendRobotcontral(urConParPub, mapAlgorithm.controlPose);
    //    printf(" %2f \n", processURData.pose.x - mapAlgorithm.controlPose.x);
    loop_rate.sleep();
  }
  return 0;
}
