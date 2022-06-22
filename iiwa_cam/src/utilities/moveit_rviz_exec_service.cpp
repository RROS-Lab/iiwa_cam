#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/ExecuteTrajectoryActionGoal.h>

#include <iiwa.hpp>

static cam::Kuka *kuka;

void moveit_callback(const moveit_msgs::ExecuteTrajectoryActionGoal &msg) {
  auto &traj = msg.goal.trajectory;
  kuka->exe_joint_traj(traj, 0.3, 1000, 1000, 1000);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "moveit_rviz_exec_service");
  ros::NodeHandle nh;

  std::string name = (argc >= 2) ? argv[1] : "iiwa";

  kuka = new cam::Kuka(name);

  ros::Subscriber moveit_sub =
      nh.subscribe("/execute_trajectory/goal", 10, moveit_callback);

  ros::Duration(2).sleep();

  ros::spin();
  return 0;
}
