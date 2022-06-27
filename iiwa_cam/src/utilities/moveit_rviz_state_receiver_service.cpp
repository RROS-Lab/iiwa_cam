#include <iiwa_msgs/JointPosition.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include <sstream>

static ros::Publisher joint_state_pub;

inline void print_vec(const std::vector<double> &vec) {
  auto vec_iter = vec.begin();
  auto vec_end_iter = vec.end();
  while (vec_iter != vec_end_iter) {
    std::cout << *vec_iter << "  ,";
    vec_iter++;
  }
  std::cout << std::endl;
}

inline void assign_position_to_vec(const iiwa_msgs::JointPosition &msg,
                           std::vector<double> &vec) {
  vec.resize(7);
  vec.at(0) = msg.position.a1;
  vec.at(1) = msg.position.a2;
  vec.at(2) = msg.position.a3;
  vec.at(3) = msg.position.a4;
  vec.at(4) = msg.position.a5;
  vec.at(5) = msg.position.a6;
  vec.at(6) = msg.position.a7;
}

void joint_state_callback(const iiwa_msgs::JointPosition &msg) {
  sensor_msgs::JointState new_msg;
  new_msg.velocity.resize(7);
  new_msg.header.stamp = ros::Time().now();

  for (int i = 1; i<=7;i++) {
    std::stringstream ss;
    ss<<"iiwa_joint_"<<i;
    
    new_msg.name.push_back(ss.str());
  }
  assign_position_to_vec(msg, new_msg.position);

  joint_state_pub.publish(new_msg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "moveit_rviz_state_receiver_service");
  std::string name = (argc >= 2) ? argv[1] : "iiwa";

  ros::NodeHandle nh;

  ros::Subscriber moveit_sub =
      nh.subscribe("/"+name+"/state/JointPosition", 100, joint_state_callback);

  joint_state_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 100);

  ros::spin();
  return 0;
}
