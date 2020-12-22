#ifndef HMC_UR_MOVE_H
#define HMC_UR_MOVE_H
#include <vector>

enum contol_mode{
  stop,
  moveJ,
  moveL,
  servol,
  force,
  contol_mode_end
};

typedef struct
{
double velocity = 1;
double acceleration = 1;
double lookaheadTime = 0.1;
double gain = 100;
}Servol_Arg;

typedef struct
{
  double velocity = 1;
  double acceleration = 1;
  std::vector<double> task_frame = {0, 0, 0, 0, 0, 0};
  std::vector<int> selection_vector = {1, 0, 0, 0, 0, 0};
  std::vector<double> wrench = {0, 0, 0, 0, 0, 0};
  int force_type = 2;
  std::vector<double> limitsSpeed = {2, 2, 2, 1, 1, 1};
}ForceMode_Arg;

typedef struct
{
const double velocity = 0.5;
const double acceleration = 0.5;
const double invertalTime = 1.0/128;
Servol_Arg servol;
ForceMode_Arg force;
std::vector<double> initJoint = {0.0, -90.0, 0.0, -90.0, 0.0, 0.0};
std::vector<double> initPose = {0.0, -194.0, 1000.0, 0.0055, 2.2262, -2.2214};
contol_mode mode = stop;
}Move_Arg;


#endif // HMC_UR_MOVE_H
