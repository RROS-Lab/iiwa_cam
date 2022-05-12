#include <iiwa_msgs/JointPosition.h>
#include <actionlib/client/simple_action_client.h>
#include <iiwa_msgs/MoveToJointPositionAction.h>
#include <iiwa_msgs/SetPTPJointSpeedLimits.h>
#include <ros/ros.h>

// #include <iiwa_ros/command/joint_position.hpp>

// #include <cstdlib>
#include <iostream>

using namespace std;

inline void assign_position_to_msg(iiwa_msgs::JointPosition& msg,
                                   const std::vector<double>& vec) {
  msg.position.a1 = vec.at(0);
  msg.position.a2 = vec.at(1);
  msg.position.a3 = vec.at(2);
  msg.position.a4 = vec.at(3);
  msg.position.a5 = vec.at(4);
  msg.position.a6 = vec.at(5);
  msg.position.a7 = vec.at(6);
}

void jointPosCtrl(iiwa_msgs::JointPosition& jointPos) {
  static string joint_pos_cmd_name = "/iiwa/command/JointPosition";
  static ros::Publisher pub =
      ros::NodeHandle().advertise<iiwa_msgs::JointPosition>(joint_pos_cmd_name,
                                                            1);
  pub.publish(jointPos);
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "joint_space_control");
  ros::NodeHandle nh;

  // option 1 in README file

  string joint_vel_srv_name = "/iiwa/configuration/setPTPJointLimits";
  string joint_pos_act_name = "/iiwa/action/move_to_joint_position";

  // service definition
  ros::ServiceClient joint_vel_client =
      nh.serviceClient<iiwa_msgs::SetPTPJointSpeedLimits>(joint_vel_srv_name);
  joint_vel_client.waitForExistence();

  // action definition
  actionlib::SimpleActionClient<iiwa_msgs::MoveToJointPositionAction>
      joint_pos_client(joint_pos_act_name);
  cout << "isServerConnected: " << joint_pos_client.isServerConnected() << endl;

  // service msg definition
  iiwa_msgs::SetPTPJointSpeedLimits joint_vel_msg;

  // action msg difinition
  iiwa_msgs::MoveToJointPositionAction joint_pos_act;

  auto& joint_pos = joint_pos_act.action_goal.goal.joint_position;
  joint_pos.position.a1 = 0;
  joint_pos.position.a2 = 0;
  joint_pos.position.a3 = 0;
  joint_pos.position.a4 = 0;
  joint_pos.position.a5 = 0;
  joint_pos.position.a6 = 0;
  joint_pos.position.a7 = 0;

  // program start
  // phase 1
  joint_vel_msg.request.joint_relative_velocity = 0.2;
  cout << "set Joint Velocity to 0.2 --> " << flush;
  joint_vel_client.call(joint_vel_msg);
  if (joint_vel_msg.response.success)
    cout << "SUCCESSFUL\n" << endl;
  else
    cout << "FAILED\n" << endl;

  // phase 2
  cout << "going home position" << endl;
  cout << "press Enter to continue ..." << endl;
  getchar();
  joint_pos_client.sendGoal(joint_pos_act.action_goal.goal);
  // joint_pos_client.waitForResult(ros::Duration(1.0));

  // phase 3
  joint_vel_msg.request.joint_relative_velocity = 0.5;
  cout << "set Joint Velocity to "
       << joint_vel_msg.request.joint_relative_velocity << " --> " << flush;
  joint_vel_client.call(joint_vel_msg);
  if (joint_vel_msg.response.success)
    cout << "SUCCESSFUL\n" << endl;
  else
    cout << "FAILED\n" << endl;

  // phase 4
  cout << "press Enter to move ..." << endl;
  // getchar();
  ros::Duration(100 * 1e-3).sleep();
  joint_pos.position.a6 = 3.1415 / 2;
  joint_pos_client.sendGoal(joint_pos_act.action_goal.goal);

  // phase 5
  cout << "press Enter to move ..." << endl;
  // getchar();
  ros::Duration(100 * 1e-3).sleep();
  joint_pos.position.a6 = 0;
  joint_pos_client.sendGoal(joint_pos_act.action_goal.goal);

  // phase 6
  cout << "press Enter to move ..." << endl;
  // getchar();
  ros::Duration(100 * 1e-3).sleep();
  joint_pos.position.a5 = 3.1415 / 2;
  joint_pos_client.sendGoal(joint_pos_act.action_goal.goal);

  // cout << "press Enter key to exit ..." << endl;
  // getchar();
  ros::shutdown();

  return 0;
}
