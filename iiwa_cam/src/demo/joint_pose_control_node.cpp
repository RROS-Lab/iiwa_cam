/**
 * @file joint_pose_control_node.cpp
 * @author Peijie Xu (peijiexu@usc.edu)
 * @brief This file shows how to control the kuka move to a joint space goal
 * @version 0.1
 * @date 2022-06-11
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <iiwa.hpp>

using namespace std;

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "joint_pose_control_node");

  // the robot name is "iiwa" (default value), should be same as
  // RoboticsAPI.data.xml on Sunrise Workspace (in Windows computer)
  std::string name = (argc >= 2) ? argv[1] : "iiwa";

  cam::Kuka kuka(name);
  vector<double> jp = {0, 0, 0, 0, 0, 0, 0};

  // phase 2
  cout << "going home position" << endl;
  cam::press_to_go();
  kuka.move_joint_ptp(jp);

  // phase 3
  kuka.set_vel_acc(0.5);

  // phase 4
  cam::press_to_go();
  jp.at(6) = 3.1415 / 2;
  kuka.move_joint_ptp(jp);

  // phase 5
  cam::press_to_go();
  jp.at(6) = 0;
  kuka.move_joint_ptp(jp);

  // phase 6
  cam::press_to_go();
  jp.at(5) = 3.1415 / 2;
  kuka.move_joint_ptp(jp);

  // phase 7
  cam::press_to_go();
  jp.at(5) = 0;
  kuka.move_joint_ptp(jp);

  // phase 8
  kuka.set_vel_acc();

  ros::shutdown();

  return 0;
}
