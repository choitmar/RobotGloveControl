#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>

#include <chrono>
#include <thread>

#include <windows.h>
#include <string>
#include <sstream>

using namespace ur_rtde;
using namespace std::chrono;

//int main(int argc, char* argv[])
//{
//  RTDEControlInterface rtde_control("172.17.170.228");
//  RTDEReceiveInterface rtde_receive("172.17.170.228");
//  std::vector<double> init_q = rtde_receive.getActualQ();
//
//  // Target in the robot base
//  std::vector<double> new_q = {0, -1.55, 1.55, 0, 1.55, 0};
//  new_q[0] += 0.2;
//
//  /**
//   * Move asynchronously in joint space to new_q, we specify asynchronous behavior by setting the async parameter to
//   * 'true'. Try to set the async parameter to 'false' to observe a default synchronous movement, which cannot be
//   * stopped by the stopJ function due to the blocking behaviour.
//   */
//  rtde_control.moveJ(new_q, 1.05, 1.4, true);
//  for (int i = 0; i < 40; i++)
//  {
//    std::this_thread::sleep_for(std::chrono::milliseconds(100));
//    std::ostringstream oss;
//    std::vector<double> target = rtde_receive.getActualTCPPose();
//    oss << "Values: " << target[0] << ", " << target[1] << ", " << target[2] << ", " << target[3] << ", " << target[4]
//        << ", "
//        << target[5] << "\n";
//    OutputDebugStringA(oss.str().c_str());  // Use OutputDebugStringA for ANSI
//  }
// 
//  // Stop the movement before it reaches new_q
//  //rtde_control.stopJ(0.5);
//
//  // Target 10 cm up in the Z-Axis of the TCP
//  //std::vector<double> target = rtde_receive.getActualTCPPose();
//  //target[2] += 0.10;
//
//  /**
//   * Move asynchronously in cartesian space to target, we specify asynchronous behavior by setting the async parameter
//   * to 'true'. Try to set the async parameter to 'false' to observe a default synchronous movement, which cannot be
//   * stopped by the stopL function due to the blocking behaviour.
//   */
//  //rtde_control.moveL(target, 0.25, 0.5, true);
//  rtde_control.moveJ(init_q, 1.05, 1.4, true);
//  for (int i = 0; i < 40; i++)
//  {
//    std::this_thread::sleep_for(std::chrono::milliseconds(100));
//    std::ostringstream oss;
//    std::vector<double> target = rtde_receive.getActualTCPPose();
//    oss << "Values: " << target[0] << ", " << target[1] << ", " << target[2] << ", " << target[3] << ", " << target[4]
//        << ", "
//        << target[5] << "\n";
//    OutputDebugStringA(oss.str().c_str());  // Use OutputDebugStringA for ANSI
//  }
//  // Stop the movement before it reaches target
//  //rtde_control.stopL(0.5);
//
//  // Move to initial joint position with a regular moveJ
//  //rtde_control.moveJ(init_q);
//
//  // Stop the RTDE control script
//  rtde_control.stopScript();
//  return 0;
//}
