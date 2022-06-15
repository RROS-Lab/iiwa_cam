#include <iiwa.hpp>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "cartesian_ptp_drop_pose_control_node");

  cam::Kuka kuka;
  kuka.set_vel_acc_drop(0.2);

  geometry_msgs::Pose pose;

  cam::press_to_go();
  pose.position.x = -0.52;
  pose.position.y = 0;
  pose.position.z = 0.18;
  pose.orientation.w = 0;
  pose.orientation.x = 0;
  pose.orientation.y = 1;
  pose.orientation.z = 0;
  kuka.move_cart_ptp_drop(pose);

  cam::press_to_go();
  pose.position.x = 0;
  pose.position.y = 0.45;
  kuka.move_cart_ptp_drop(pose);

  cam::press_to_go();
  kuka.move_cart_ptp_drop(-0.44, 0, 1.02, 0.8776, 0, -0.4794, 0);


  ros::shutdown();
  return 0;
}