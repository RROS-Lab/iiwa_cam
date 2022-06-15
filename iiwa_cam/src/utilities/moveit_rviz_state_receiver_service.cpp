#include <iiwa_msgs/JointPositionVelocity.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <cmath>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

inline void print_vec(const std::vector<double> &vec) {
  auto vec_iter = vec.begin();
  auto vec_end_iter = vec.end();
  while (vec_iter != vec_end_iter) {
    std::cout << *vec_iter << "  ,";
    vec_iter++;
  }
  std::cout << std::endl;
}

inline void assign_msg_to_vec(const iiwa_msgs::JointQuantity &msg,
                              std::vector<double> &vec) {
  vec.insert(vec.begin(),
             {msg.a1, msg.a2, msg.a3, msg.a4, msg.a5, msg.a6, msg.a7});
}

void joint_state_callback(iiwa_msgs::JointPositionVelocityConstPtr msg,
                          ros::Publisher &joint_state_pub) {
  sensor_msgs::JointState new_msg;
  new_msg.header = msg->header;

  for (int i = 1; i <= 7; i++) {
    std::stringstream ss;
    ss << "iiwa_joint_" << i;

    new_msg.name.push_back(ss.str());
  }
  assign_msg_to_vec(msg->position, new_msg.position);
  assign_msg_to_vec(msg->velocity, new_msg.velocity);

  joint_state_pub.publish(new_msg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "moveit_rviz_state_receiver_service");

  ros::NodeHandle nh;

  std::string name = (argc >= 2) ? argv[1] : "iiwa";
  std::string rob_ns = "/" + name;

  auto joint_state_pub =
      nh.advertise<sensor_msgs::JointState>("/joint_states", 100);

  auto callback = boost::bind(&joint_state_callback, _1, joint_state_pub);

  ros::Subscriber moveit_sub = nh.subscribe<iiwa_msgs::JointPositionVelocity>(
      rob_ns + "/state/JointPositionVelocity", 100, callback);

  std::cout << "MoveIt!-RViz Robot State Receiver is monitoring: " << name
            << std::endl;

  ros::spin();

  return 0;
}
