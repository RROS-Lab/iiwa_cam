#pragma once
#include <actionlib/client/simple_action_client.h>
#include <iiwa_msgs/JointPosition.h>
#include <iiwa_msgs/MoveAlongSplineAction.h>
#include <iiwa_msgs/MoveToCartesianPoseAction.h>
#include <iiwa_msgs/MoveToJointPositionAction.h>
#include <iiwa_msgs/SetPTPCartesianSpeedLimits.h>
#include <iiwa_msgs/SetPTPJointSpeedLimits.h>
#include <iiwa_msgs/SetSmartServoJointSpeedLimits.h>
#include <iiwa_msgs/SetSmartServoLinSpeedLimits.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <ros/ros.h>

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace cam {

/**
 * @brief read cartesian trajectory data from a file specified by file_name,
 * solve the trajectory in the vector traj
 *
 * @param file
 * @param traj
 */
void read_traj_cart(char const *file, std::vector<geometry_msgs::Pose> &traj) {
  std::ifstream fp(file);
  std::string line;
  getline(fp, line);  // ignore the first line of csv file

  while (getline(fp, line)) {
    std::vector<double> data_line;
    std::istringstream readstr(line);

    for (int j = 0; j < 7; j++) {
      std::string number;
      getline(readstr, number, ',');
      data_line.emplace_back(std::stod(number));
    }

    geometry_msgs::Pose pose;
    pose.position.x = data_line.at(0);
    pose.position.y = data_line.at(1);
    pose.position.z = data_line.at(2);
    pose.orientation.w = data_line.at(3);
    pose.orientation.x = data_line.at(4);
    pose.orientation.y = data_line.at(5);
    pose.orientation.z = data_line.at(6);

    traj.emplace_back(pose);
  }
}

inline void press_to_go() {
  static int cnt = 0;
  ROS_INFO("break point %d: Press Enter to continue", ++cnt);
  getchar();
}

inline void print_vec(const std::vector<double> &vec) {
  auto vec_iter = vec.begin();
  auto vec_end_iter = vec.end();
  while (vec_iter != vec_end_iter) {
    std::cout << *vec_iter << ", ";
    vec_iter++;
  }
  std::cout << std::endl;
}
}  // namespace cam

namespace cam {

class Kuka;

class Frame {
 private:
  friend class Kuka;

  int joints;
  std::vector<double> joint_pos;
  std::vector<double> cart_pos = std::vector<double>(7);  // X Y Z [W X Y Z]

 public:
  Frame() = default;
  Frame(int total_joints) : joints(total_joints), joint_pos(total_joints) {}

  const std::vector<double> &get_cartesian_pos() const { return cart_pos; }
  const std::vector<double> &get_joint_pos() const { return joint_pos; }

  bool set_joint_pos(const std::vector<double> &joint_position) {
    if (joint_position.size() != joints)
      return false;
    joint_pos = std::move(joint_position);
    return true;
  }
};

class Kuka {
 private:
  ros::ServiceClient cart_spline_vel_client;
  ros::ServiceClient joint_vel_client;
  ros::ServiceClient ss_joint_vel_client;
  ros::ServiceClient ss_lin_vel_client;

  ros::Publisher joint_spline_pub;
  ros::Publisher joint_pos_droppable_pub;
  ros::Publisher cartesian_pos_droppable_pub;

  actionlib::SimpleActionClient<iiwa_msgs::MoveToJointPositionAction>
      *joint_pos_client;

  actionlib::SimpleActionClient<iiwa_msgs::MoveToCartesianPoseAction>
      *cartesian_pos_ptp_client;

  actionlib::SimpleActionClient<iiwa_msgs::MoveToCartesianPoseAction>
      *cartesian_pos_lin_client;

  actionlib::SimpleActionClient<iiwa_msgs::MoveAlongSplineAction>
      *cartesian_spline_client;

  double m_joint_vel = 0;
  double m_joint_acc = 0;

  double m_cart_pos_vel = 0;
  double m_cart_ori_vel = 0;
  double m_cart_pos_acc = 0;
  double m_cart_ori_acc = 0;
  double m_cart_pos_jerk = 0;
  double m_cart_ori_jerk = 0;

  double m_joint_vel_drop = 0;
  double m_joint_acc_drop = 0;
  double m_joint_over_acc_drop = 0;

  std::string iiwa_name;

 public:
  Kuka() { rob_init("iiwa"); }
  Kuka(std::string rob_name) { rob_init(rob_name); }

  void rob_init(const std::string &rob_name) {
    ros::NodeHandle nh;

    std::stringstream ss("/");
    ss << rob_name;

    iiwa_name = std::move(ss.str());

    cart_spline_vel_client =
        nh.serviceClient<iiwa_msgs::SetPTPCartesianSpeedLimits>(
            iiwa_name + "/configuration/setPTPCartesianLimits");

    joint_vel_client = nh.serviceClient<iiwa_msgs::SetPTPJointSpeedLimits>(
        iiwa_name + "/configuration/setPTPJointLimits");

    ss_joint_vel_client =
        nh.serviceClient<iiwa_msgs::SetSmartServoJointSpeedLimits>(
            iiwa_name + "/configuration/setSmartServoLimits");

    ss_lin_vel_client =
        nh.serviceClient<iiwa_msgs::SetSmartServoLinSpeedLimits>(
            iiwa_name + "/configuration/setSmartServoLinLimits");

    joint_pos_client =
        new actionlib::SimpleActionClient<iiwa_msgs::MoveToJointPositionAction>(
            iiwa_name + "/action/move_to_joint_position");

    cartesian_pos_ptp_client =
        new actionlib::SimpleActionClient<iiwa_msgs::MoveToCartesianPoseAction>(
            iiwa_name + "/action/move_to_cartesian_pose");

    cartesian_pos_lin_client =
        new actionlib::SimpleActionClient<iiwa_msgs::MoveToCartesianPoseAction>(
            iiwa_name + "/action/move_to_cartesian_pose_lin");

    cartesian_spline_client =
        new actionlib::SimpleActionClient<iiwa_msgs::MoveAlongSplineAction>(
            iiwa_name + "/action/move_along_spline");

    joint_spline_pub =
        nh.advertise<iiwa_msgs::Spline>(iiwa_name + "/command/JointSpline", 1);

    joint_pos_droppable_pub = nh.advertise<iiwa_msgs::JointPosition>(
        iiwa_name + "/command/JointPosition", 1);

    cartesian_pos_droppable_pub = nh.advertise<geometry_msgs::PoseStamped>(
        iiwa_name + "/command/CartesianPose", 1);

    set_vel_acc();
    set_cart_traj_vel_acc();
  }

  ~Kuka() {
    delete joint_pos_client;
    delete cartesian_pos_lin_client;
    delete cartesian_pos_ptp_client;
    delete cartesian_spline_client;
  }

  /**
   * @brief Get the saved frames from teaching pendant
   *
   */
  void get_recorded_frames() {  // TODO
  }

  void set_vel_acc_drop(const double vel = 0.1, const double acc = 0.1,
                        const double override_acc = 1.0) {
    if (vel == m_joint_vel_drop && acc == m_joint_acc_drop &&
        override_acc == m_joint_over_acc_drop) {
      return;
    } else {
      m_joint_vel_drop = vel;
      m_joint_acc_drop = acc;
      m_joint_over_acc_drop = override_acc;
    }
    // service msg definition
    static iiwa_msgs::SetSmartServoJointSpeedLimits joint_vel_msg;

    // set joint velocity limit
    joint_vel_msg.request.joint_relative_velocity = vel;
    joint_vel_msg.request.joint_relative_acceleration = acc;
    joint_vel_msg.request.override_joint_acceleration = override_acc;


    std::cout << "set droppable Joint Velocity to " << vel << " --> " << std::flush;
    ss_joint_vel_client.call(joint_vel_msg);
    std::cout << (joint_vel_msg.response.success ? "SUCCESSFUL" : "FAILED")
              << std::endl;
  }

  /**
   * @brief Set the velocity and accelaration of joint space. This function
   * controls the velocity and accelaration of move_joint_ptp(), move_cart_ptp()
   *
   * @param vel default = 0.1
   * @param acc default = 0.1
   */
  void set_vel_acc(const double vel = 0.1, const double acc = 0.1) {
    if (vel == m_joint_vel && acc == m_joint_acc) {
      return;
    } else {
      m_joint_vel = vel;
      m_joint_acc = acc;
    }
    // service msg definition
    static iiwa_msgs::SetPTPJointSpeedLimits joint_vel_msg;

    // set joint velocity limit
    joint_vel_msg.request.joint_relative_velocity = vel;
    joint_vel_msg.request.joint_relative_acceleration = acc;

    std::cout << "set Joint Velocity to " << vel << " --> " << std::flush;
    joint_vel_client.call(joint_vel_msg);
    std::cout << (joint_vel_msg.response.success ? "SUCCESSFUL" : "FAILED")
              << std::endl;
  }

  /**
   * @brief Set the velocity, accelaration, and jerk in cartesian space. This
   * function controls the velocity, accelaration, and jerk of exe_cart_traj()
   *
   * @param maxCartesianVelocity default = 0.1
   * @param maxOrientationVelocity default = 0.5
   * @param maxCartesianAcceleration default = 0.2
   * @param maxOrientationAcceleration default = 0.1
   * @param maxCartesianJerk default = -1
   * @param maxOrientationJerk default = -1
   */
  void set_cart_traj_vel_acc(const double maxCartesianVelocity = 0.1,
                             const double maxOrientationVelocity = 0.5,
                             const double maxCartesianAcceleration = 0.2,
                             const double maxOrientationAcceleration = 0.1,
                             const double maxCartesianJerk = -1.0,
                             const double maxOrientationJerk = -1.0) {
    if (m_cart_pos_vel == maxCartesianVelocity &&
        m_cart_ori_vel == maxOrientationVelocity &&
        m_cart_pos_acc == maxCartesianAcceleration &&
        m_cart_ori_acc == maxOrientationAcceleration &&
        m_cart_pos_jerk == maxCartesianJerk &&
        m_cart_ori_jerk == maxOrientationJerk) {
      return;
    } else {
      m_cart_pos_vel = maxCartesianVelocity;
      m_cart_ori_vel = maxOrientationVelocity;
      m_cart_pos_acc = maxCartesianAcceleration;
      m_cart_ori_acc = maxOrientationAcceleration;
      m_cart_pos_jerk = maxCartesianJerk;
      m_cart_ori_jerk = maxOrientationJerk;
    }

    iiwa_msgs::SetPTPCartesianSpeedLimits cart_vel_msg;

    cart_vel_msg.request.maxCartesianVelocity = maxCartesianVelocity;
    cart_vel_msg.request.maxCartesianAcceleration = maxCartesianAcceleration;
    cart_vel_msg.request.maxCartesianJerk = maxCartesianJerk;
    cart_vel_msg.request.maxOrientationVelocity = maxOrientationVelocity;
    cart_vel_msg.request.maxOrientationAcceleration =
        maxOrientationAcceleration;
    cart_vel_msg.request.maxOrientationJerk = maxOrientationJerk;

    cart_spline_vel_client.call(cart_vel_msg);

    std::cout << "set Cartesian PTP limits: \n"
              << maxCartesianVelocity << ", " << maxOrientationVelocity << ", "
              << maxCartesianAcceleration << ", " << maxOrientationAcceleration
              << ", " << maxCartesianJerk << ", " << maxOrientationJerk << " "
              << "--> "
              << (cart_vel_msg.response.success ? "SUCCESSFUL" : "FAILED")
              << std::endl;
  }

  /**
   * @brief Move kuka point to point (PTP) assigned by joint space goal. The
   * unit of joint position is radiant
   *
   * @param vec
   * @param sleep_time default = 500 ms
   */
  void move_joint_ptp(const std::vector<double> &vec,
                      const double sleep_time = 500.0) {
    move_joint_ptp(vec.at(0), vec.at(1), vec.at(2), vec.at(3), vec.at(4),
                   vec.at(5), vec.at(6), sleep_time);
  }

  /**
   * @brief Move kuka point to point (PTP) assigned by joint space goal. The
   * unit of joint position is radiant
   *
   * @param joint_positions j1 ~ j7
   * @param sleep_time default = 500 ms
   */
  void move_joint_ptp(const double j1, const double j2, const double j3,
                      const double j4, const double j5, const double j6,
                      const double j7, const double sleep_time = 500.0) {
    iiwa_msgs::MoveToJointPositionAction joint_pos_act;
    auto &joint_pos = joint_pos_act.action_goal.goal.joint_position;
    joint_pos.position.a1 = j1;
    joint_pos.position.a2 = j2;
    joint_pos.position.a3 = j3;
    joint_pos.position.a4 = j4;
    joint_pos.position.a5 = j5;
    joint_pos.position.a6 = j6;
    joint_pos.position.a7 = j7;
    joint_pos_client->sendGoal(joint_pos_act.action_goal.goal);
    ros::Duration(sleep_time * 1e-3).sleep();
  }

  /**
   * @brief Move kuka point to point (PTP) assigned by joint space goal. The
   * unit of joint position is radiant. The robot will abandon previous goal
   * when it receive a new goal, even when it is still executing.
   *
   * @param vec
   */
  void move_joint_ptp_drop(const std::vector<double> &vec) {
    move_joint_ptp_drop(vec.at(0), vec.at(1), vec.at(2), vec.at(3), vec.at(4),
                        vec.at(5), vec.at(6));
  }

  /**
   * @brief Move kuka point to point (PTP) assigned by joint space goal. The
   * unit of joint position is radiant. The robot will abandon previous goal
   * when it receive a new goal, even when it is still executing.
   *
   * @param joint_positions j1 ~ j7
   */
  void move_joint_ptp_drop(const double j1, const double j2, const double j3,
                           const double j4, const double j5, const double j6,
                           const double j7) {
    iiwa_msgs::JointPosition joint_pos;
    joint_pos.position.a1 = j1;
    joint_pos.position.a2 = j2;
    joint_pos.position.a3 = j3;
    joint_pos.position.a4 = j4;
    joint_pos.position.a5 = j5;
    joint_pos.position.a6 = j6;
    joint_pos.position.a7 = j7;
    joint_pos_droppable_pub.publish(joint_pos);
  }

  /**
   * @brief Move kuka point to point (PTP) assigned by cartesian space goal. The
   * unit of cartesian position is meter
   *
   * @param pose
   * @param sleep_time default = 500 ms
   */
  void move_cart_ptp(const geometry_msgs::Pose &pose,
                     const double sleep_time = 500.0) {
    // action msg difinition
    iiwa_msgs::MoveToCartesianPoseAction cartesian_pos_act;

    // set goal
    auto &poseStamped =
        cartesian_pos_act.action_goal.goal.cartesian_pose.poseStamped;
    cartesian_pos_act.action_goal.goal.cartesian_pose.redundancy.status = 5;
    // set frame id (important!)
    poseStamped.header.frame_id = "iiwa_link_0";

    // set cartesian position
    poseStamped.pose = pose;

    cartesian_pos_ptp_client->sendGoal(cartesian_pos_act.action_goal.goal);
    // cartesian_pos_client.sendGoalAndWait();

    ros::Duration(sleep_time * 1e-3).sleep();
  }

  /**
   * @brief Move kuka point to point (PTP) assigned by cartesian space goal. The
   * unit of cartesian position is meter
   *
   * @param pose cartesian position X Y Z w x y z
   * @param sleep_time default = 500 ms
   */
  void move_cart_ptp(const double posX, const double posY, const double posZ,
                     const double oriW, const double oriX, const double oriY,
                     const double oriZ, const double sleep_time = 500.0) {
    geometry_msgs::Pose pose;
    pose.position.x = posX;
    pose.position.y = posY;
    pose.position.z = posZ;
    pose.orientation.w = oriW;
    pose.orientation.x = oriX;
    pose.orientation.y = oriY;
    pose.orientation.z = oriZ;
    move_cart_ptp(pose, sleep_time);
  }

  /**
   * @brief Move kuka point to point (PTP) assigned by cartesian space goal. The
   * unit of cartesian position is meter. The robot will abandon previous goal
   * when it receive a new goal, even when it is still executing.
   *
   * @param pose cartesian position X Y Z w x y z
   * @param sleep_time default = 500 ms
   */
  void move_cart_ptp_drop(const geometry_msgs::Pose &pose) {
    geometry_msgs::PoseStamped cart_pos;

    // set cartesian position
    cart_pos.pose = pose;

    cartesian_pos_droppable_pub.publish(cart_pos);
  }

  /**
   * @brief Move kuka point to point (PTP) assigned by cartesian space goal. The
   * unit of cartesian position is meter. The robot will abandon previous goal
   * when it receive a new goal, even when it is still executing.
   *
   * @param pose cartesian position X Y Z w x y z
   * @param sleep_time default = 500 ms
   */
  void move_cart_ptp_drop(const double posX, const double posY,
                          const double posZ, const double oriW,
                          const double oriX, const double oriY,
                          const double oriZ) {
    geometry_msgs::Pose pose;
    pose.position.x = posX;
    pose.position.y = posY;
    pose.position.z = posZ;
    pose.orientation.w = oriW;
    pose.orientation.x = oriX;
    pose.orientation.y = oriY;
    pose.orientation.z = oriZ;
    move_cart_ptp_drop(pose);
  }

  /**
   * @brief Move kuka point to point (LIN) assigned by cartesian space goal.
   * The unit of cartesian position is meter
   *
   * @param pose
   * @param sleep_time default = 500 ms
   */
  void move_cart_lin(geometry_msgs::Pose pose,
                     const double sleep_time = 500.0) {
    move_cart_lin(pose.position.x, pose.position.y, pose.position.z,
                  pose.orientation.w, pose.orientation.x, pose.orientation.y,
                  pose.orientation.z, sleep_time);
  }

  /**
   * @brief Move kuka linearly (LIN) assigned by cartesian space goal. The
   * unit of cartesian position is meter
   *
   * @param sleep_time default = 500 ms
   */
  void move_cart_lin(const double posX, const double posY, const double posZ,
                     const double oriW, const double oriX, const double oriY,
                     const double oriZ, const double sleep_time = 500.0) {
    // action msg difinition
    iiwa_msgs::MoveToCartesianPoseAction cartesian_pos_act;

    cartesian_pos_act.action_goal.goal.cartesian_pose.redundancy.status = 5;

    // set goal
    auto &poseStamped =
        cartesian_pos_act.action_goal.goal.cartesian_pose.poseStamped;

    // set frame id (important!)
    poseStamped.header.frame_id = "iiwa_link_0";

    // set cartesian position
    poseStamped.pose.position.x = posX;
    poseStamped.pose.position.y = posY;
    poseStamped.pose.position.z = posZ;
    poseStamped.pose.orientation.w = oriW;
    poseStamped.pose.orientation.x = oriX;
    poseStamped.pose.orientation.y = oriY;
    poseStamped.pose.orientation.z = oriZ;

    cartesian_pos_lin_client->sendGoal(cartesian_pos_act.action_goal.goal);

    ros::Duration(sleep_time * 1e-3).sleep();
  }

  /**
   * @brief Move robot along a trajectory in cartesian space
   *
   * @param trajectory
   */
  void exe_cart_traj(const std::vector<geometry_msgs::Pose> &trajectory) {
    iiwa_msgs::MoveAlongSplineAction spline_act_msg;

    spline_act_msg.action_goal.header.frame_id = "iiwa_link_0";

    auto &seg_vec = spline_act_msg.action_goal.goal.spline.segments;

    size_t traj_len = trajectory.size();

    seg_vec.reserve(traj_len);

    for (auto iter = trajectory.begin(); iter != trajectory.end(); iter++) {
      iiwa_msgs::SplineSegment seg;
      seg.type = iiwa_msgs::SplineSegment::SPL;
      seg.point.poseStamped.header.frame_id = "iiwa_link_0";

      seg.point.poseStamped.pose = *iter;

      seg_vec.emplace_back(seg);
    }

    move_cart_ptp(trajectory[0]);

    cartesian_spline_client->sendGoal(spline_act_msg.action_goal.goal);
  }

  /**
   * @brief Move robot along a trajectory in joint space
   *
   * @param trajectory
   */
  void exe_joint_traj(const moveit_msgs::RobotTrajectory &trajectory) {
    iiwa_msgs::Spline spline_msg;
    const auto &traj_vec = trajectory.joint_trajectory.points;

    unsigned traj_size = traj_vec.size();

    ROS_INFO("spline traj size:  %u", traj_size);

    spline_msg.segments.reserve(traj_size);

    auto traj_vec_iter = traj_vec.begin();
    auto traj_vec_end_iter = traj_vec.end();
    while (traj_vec_iter != traj_vec_end_iter) {
      iiwa_msgs::SplineSegment seg;

      seg.point.poseStamped.pose.position.x = traj_vec_iter->positions.at(0);
      seg.point.poseStamped.pose.position.y = traj_vec_iter->positions.at(1);
      seg.point.poseStamped.pose.position.z = traj_vec_iter->positions.at(2);
      seg.point.poseStamped.pose.orientation.w = traj_vec_iter->positions.at(3);
      seg.point.poseStamped.pose.orientation.x = traj_vec_iter->positions.at(4);
      seg.point.poseStamped.pose.orientation.y = traj_vec_iter->positions.at(5);
      seg.point.poseStamped.pose.orientation.z = traj_vec_iter->positions.at(6);

      spline_msg.segments.emplace_back(seg);
      traj_vec_iter++;
    }
    // spline_msg.segments.pop_back();
    // press_to_go();

    joint_spline_pub.publish(spline_msg);
  }
};

}  // namespace cam