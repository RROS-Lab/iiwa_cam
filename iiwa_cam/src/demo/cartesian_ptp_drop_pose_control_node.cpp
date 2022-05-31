#include <cam.hpp>


int main(int argc, char *argv[]) {
  ros::init(argc, argv, "cartesian_ptp_drop_pose_control_node");

  cam::Kuka kuka;
  kuka.set_vel_acc_drop(0.2);

  geometry_msgs::Pose pose;


  cam::press_to_go();
  pose.position.x = -0.52;
  pose.position.y = 0;
  pose.position.z = 0.15;
  pose.orientation.w = 0;
  pose.orientation.x = 0;
  pose.orientation.y = 1;
  pose.orientation.z = 0;
  kuka.move_cart_ptp_drop(pose);

  cam::press_to_go();
  pose.position.x = 0;
  pose.position.y = 0.4;
  kuka.move_cart_ptp_drop(pose);

  cam::press_to_go();
  pose.position.x = -0.44;
  pose.position.y = 0;
  pose.position.z = 1.02;
  pose.orientation.w = 0.877597987651825;
  pose.orientation.x = 0;
  pose.orientation.y = -0.4793973364340466;
  pose.orientation.z = 0;
  kuka.move_cart_ptp_drop(pose);

  std::cout << "ROS start spinning now ... Press Ctrl+C to stop" << std::endl;
  ros::spin();

  ros::shutdown();
  return 0;
}