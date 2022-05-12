#include <moveit/move_group_interface/move_group_interface.h>
#include <ros/ros.h>

#include <cam.hpp>
#include <cmath>

int main(int argc, char **argv) {
  ros::init(argc, argv, "moveit_plan_and_exec_node");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit::planning_interface::MoveGroupInterface *arm_ptr = NULL;

  if (argc < 2) {
    // you need to run demo.lauch from iiwa_cam_moveit pkg 
    // before running this program
    arm_ptr = new moveit::planning_interface::MoveGroupInterface("iiwa_arm");
  } else {
    // 1. you need to run moveit_planning_execution.launch from iiwa_moveit pkg
    //    before this program
    // 2. when you run this program, add argument:
    //    ns:=iiwa
    moveit::planning_interface::MoveGroupInterface::Options opt(
        "manipulator", "/iiwa/robot_description");
    arm_ptr = new moveit::planning_interface::MoveGroupInterface(opt);
  }
  auto &arm = *arm_ptr;

  cam::Kuka kuka;

  std::string end_effector_link = arm.getEndEffectorLink();
  std::string reference_frame = "iiwa_link_0";
  arm.setPoseReferenceFrame(reference_frame);

  arm.allowReplanning(true);

  arm.setGoalPositionTolerance(0.001);
  arm.setGoalOrientationTolerance(0.01);

  arm.setMaxAccelerationScalingFactor(0.8);
  arm.setMaxVelocityScalingFactor(0.8);

  kuka.move_cart_ptp(-0.52, 0, 0.15, 0, 0, 1, 0);
  geometry_msgs::Pose target_pose;
  target_pose.orientation.x = 0;
  target_pose.orientation.y = 1;
  target_pose.orientation.z = 0;
  target_pose.orientation.w = 0;

  target_pose.position.x = -0.52;
  target_pose.position.y = 0.0;
  target_pose.position.z = 0.15;

  std::vector<geometry_msgs::Pose> waypoints;

  double centerA = target_pose.position.x;
  double centerB = target_pose.position.y;
  double radius = 0.1;

  for (double th = 0.0; th < 6.28; th = th + 0.01) {
    target_pose.position.x = centerA + radius * cos(th);
    target_pose.position.y = centerB + radius * sin(th);
    waypoints.emplace_back(target_pose);
  }

  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = 0.0;
  int maxtries = 100;
  int attempts = 0;

  while (fraction < 1.0 && attempts < maxtries) {
    fraction = arm.computeCartesianPath(waypoints, eef_step, jump_threshold,
                                        trajectory);
    attempts++;
  }

  if (fraction < 0.5) {
    ROS_INFO("Path planning failed with only %0.6f success after %d attempts.",
             fraction, maxtries);
  } else {
    if (fraction < 1.0)
      ROS_INFO("Path planning failed with %0.6f success after %d attempts.",
               fraction, maxtries);
    else
      ROS_INFO("Path computed successfully. Moving the arm.");

    kuka.exe_joint_traj(trajectory);
  }

  ros::shutdown();
  return 0;
}
