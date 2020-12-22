#ifndef DATA_PROCESSING_H
#define DATA_PROCESSING_H
#include <vector>
#include <queue>
#include <float.h>

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
double x = 0.0;
double y = 0.0;
double z = 0.0;
double Rx = 0.0;
double Ry = 0.0;
double Rz = 0.0;
}Robot_Pose;

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
  Plane_Fun fun;
  bool buildInSpace = false;
  double rho = 0.01;
  double lambda = 1;
  plane_VF_arithmetic arithmetic = Trigonometric;
  double distance = DBL_MAX;
  double direction = 0.0; //The direction of point to target plane
  double virtualForce = 0.0;
  Point unitNormalVector;
}Plane;




#endif // DATA_PROCESSING_H
