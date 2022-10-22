/**
 * @file cartesian_spline_node.cpp
 * @author Peijie Xu (peijiexu@usc.edu)
 * @brief This file shows how to control the kuka to execute a cartesian space
 * trajectory, including cartesian velocity control and real path recording.
 * @version 0.1
 * @date 2022-06-11
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <cmath>
#include <iiwa.hpp>

int main(int argc, char **argv) {
  ros::init(argc, argv, "cartesian_spline_node");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  // the robot name is "iiwa" (default value), should be same as
  // RoboticsAPI.data.xml on Sunrise Workspace (in Windows computer)
  std::string name = "iiwa";

  cam::Kuka kuka(name);

  // print kuka message
  kuka.set_printer(true);

  // set cartesian trajectory velocity to 0.1
  kuka.set_cart_traj_vel_acc(0.1, 0.1);

  // the Kuka::exe_cart_traj() method needs 2 parameters
  std::vector<geometry_msgs::Pose> waypoints;
  std::vector<int> status;

  if (argc > 1) {
    // read cartesian trajectory from file
    std::cout << "using data from file: " << argv[1] << std::endl;
    cam::read_cart_traj(argv[1], waypoints, status);

  } else {
    geometry_msgs::Pose target_pose;
    target_pose.orientation.x = 0;
    target_pose.orientation.y = 1;
    target_pose.orientation.z = 0;
    target_pose.orientation.w = 0;

    target_pose.position.x = 0.52;
    target_pose.position.y = 0.0;
    target_pose.position.z = 0.15;

    double centerA = target_pose.position.x;
    double centerB = target_pose.position.y;
    double radius = 0.1;

    for (double th = 0.0; th < 6.28; th = th + 0.01) {
      target_pose.position.x = centerA + radius * cos(th);
      target_pose.position.y = centerB + radius * sin(th);
      waypoints.emplace_back(target_pose);
    }
    status.resize(waypoints.size(), 2);
  }
  // start recording end effector state
  kuka.end_effector_state().start_recording();

  // execute cartesian space trajectory
  kuka.exe_cart_traj(waypoints, status);

  // stop recording end effector state
  kuka.end_effector_state().end_recording();
  return 0;
}
