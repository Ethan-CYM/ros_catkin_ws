#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>

#include <thread>
#include <chrono>

using namespace ur_rtde;
using namespace std::chrono;

int main(int argc, char* argv[])
{
  RTDEControlInterface rtde_control("127.0.0.1");
  RTDEReceiveInterface rtde_receive("127.0.0.1");
  std::vector<double> init_q = rtde_receive.getActualQ();

  // Stop the RTDE control script
  rtde_control.stopScript();
  return 0;
}