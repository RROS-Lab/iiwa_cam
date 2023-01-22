#include <iiwa_cam/PyCartesianPose.h>
#include <iiwa_cam/PyCartesianSpline.h>
#include <iiwa_cam/PyJointRelVel.h>

#include <iiwa.hpp>

static cam::Kuka *kuka;

/**
 * @brief Callback function for
 * @param req
 * @param res
 * @return true once the reqest is served
 */
bool cart_spline_cb(iiwa_cam::PyCartesianSpline::Request &req, iiwa_cam::PyCartesianSpline::Response &res) {
  kuka->exe_cart_traj(req.trajectory, req.status);
  res.success = true;

  return true;
}

/**
 * @brief Callback function for
 * @param req
 * @param res
 * @return true once the reqest is served
 */
bool cart_ptp_drop_cb(iiwa_cam::PyCartesianPose::Request &req, iiwa_cam::PyCartesianPose::Response &res) {
  if (req.linearMotion)
    kuka->move_cart_lin_drop(req.pose);
  else
    kuka->move_cart_ptp_drop(req.pose);
  res.success = true;

  return true;
}

/**
 * @brief Callback function for Set the vel acc drop object
 * 
 * @param req 
 * @param res 
 * @return true 
 * @return false 
 */
bool set_vel_acc_drop(iiwa_cam::PyJointRelVel::Request &req, iiwa_cam::PyJointRelVel::Response &res) {
  
  if(req.joint_relative_velocity>0.2){
    ROS_WARN("The Robot will move at fast velocities");
  }

  kuka->set_vel_acc_drop(req.joint_relative_velocity, req.joint_relative_acceleration);


  res.success = true;

  return true;

}

bool set_vel_acc(iiwa_cam::PyJointRelVel::Request &req, iiwa_cam::PyJointRelVel::Response &res) {
  ROS_INFO_STREAM("I the Service!!!!!");
  if(req.joint_relative_velocity>0.1){
    ROS_WARN("The Robot will move at fast velocities");
  }
  kuka->set_vel_acc(req.joint_relative_velocity, req.joint_relative_acceleration);


  res.success = true;

  return true;

}

int main(int argc, char **argv) {
  ros::init(argc, argv, "python_control_service");
  ros::NodeHandle nh;

  std::string name = (argc >= 2) ? argv[1] : "iiwa";

  kuka = new cam::Kuka(name);
  kuka->set_vel_acc_lin_drop(0.1);
  kuka->set_vel_acc_drop(0.2,0.1);

  ros::ServiceServer cart_ptp_drop_srv = nh.advertiseService("/cam/iiwa/command/CartesianPose", cart_ptp_drop_cb);
  ros::ServiceServer cart_spline_srv = nh.advertiseService("/cam/iiwa/command/CartesianSpline", cart_spline_cb);

  ros::ServiceServer set_vel_acc_drop_srv = nh.advertiseService("/cam/iiwa/command/setJointRelVelAccDrop", set_vel_acc_drop);
  ros::ServiceServer set_vel_acc_srv = nh.advertiseService("/cam/iiwa/command/setJointRelVelAcc", set_vel_acc);

  ros::spin();
  std::cout << "\nShut down Python Control Service" << std::endl;
  // do cleaning
  ros::shutdown();

  return 0;
}
