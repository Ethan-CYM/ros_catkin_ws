#ifndef DATA_PROCESSING_H
#define DATA_PROCESSING_H
//#include <vector>
//#include <queue>
#include <deque>
#include <float.h>
#include <Eigen/Dense>

#define CTRL_FREQ 128

typedef struct
{
  double fx = 0.0;
  double fy = 0.0;
  double fz = 0.0;
  double tx = 0.0;
  double ty = 0.0;
  double tz = 0.0;

}FT_Data;

typedef struct
{
  bool touchFTFlag = false;
  bool forceFlag = false;
  bool FTNeedCalibration = true;
  unsigned long FTDataBuffer = 50;

  Eigen::Matrix<double,3,1> force;
  Eigen::Matrix<double,3,1> torque;
  Eigen::Matrix<double,3,1> meanForce;
  Eigen::Matrix<double,3,1> meanTorque;
  Eigen::Matrix<double,3,1> forceErro;
  Eigen::Matrix<double,3,1> torqueErro;
  std::deque<Eigen::Matrix<double,3,1>> forceDeque;
  std::deque<Eigen::Matrix<double,3,1>> torqueDeque;
}FT_Args;

typedef struct
{
double x = 0.0;
double y = 0.0;
double z = 0.0;
double Rx = 0.0;
double Ry = 0.0;
double Rz = 0.0;
}Robot_Pose;

typedef struct
{
Eigen::Matrix<double,3,1> position;
Eigen::Matrix<double,3,1> posture;
Eigen::Matrix<double,6,1> actualTCPPose;
Eigen::Matrix<double,6,1> targetTCPPose;
Eigen::Matrix<double,6,1> actualTCPSpeed;
Eigen::Matrix<double,6,1> targetTCPSpeed;
Eigen::Matrix<double,6,1> actualTCPForce;
Eigen::Matrix<double,3,1> actualToolAccel;
}Robot_Args;

typedef struct
{
  double K1 = 0.0; //弹簧的劲度系数倒数
  double K2 = -0.3; //阻尼
  double K3 = 0.1; // 质量的倒数
  double K4 = 0.2; //静摩擦力
}Controller_Args;

typedef struct
{
double x = 0.0;
double y = 0.0;
double z = 0.0;
}Point;

enum plane_VF_arithmetic{
  Trigonometric,
  HalfTrigonometric,
  Power,
  plane_VFF_arithmetic_end
};

enum target_attribute{
  plane,
  target_attribute_end
};


typedef struct
{
  //A(x-x0) + B(y - y0) + C(z - z0) = 0
  double A = 0.0;
  double B = 0.0;
  double C = 0.0;
  double x0 = 0.0;
  double y0 = 0.0;
  double z0 = 0.0;
}Plane_Fun;

typedef struct
{
  //A(x-x0) + B(y - y0) + C(z - z0) = 0
  Eigen::Matrix<double,3,1> normalVector;//A,B,C
  Eigen::Matrix<double,3,1> P0;//x0,y0,z0
}Plane_Fun_M;

typedef struct
{
//  Plane_Fun fun;
  Plane_Fun_M funM;
  bool buildInSpace = false;
  double rho = 0.01;
  double lambda = 1;
  plane_VF_arithmetic arithmetic = Trigonometric;
  double distance = DBL_MAX;
  double direction = 0.0; //The direction of point to target plane
  double virtualForce = 0.0;
  Point unitNormalVector;
  Eigen::Matrix<double,3,1> unitNormalVectorM;
}Plane;




#endif // DATA_PROCESSING_H
