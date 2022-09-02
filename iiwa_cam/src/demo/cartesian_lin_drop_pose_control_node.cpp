
#include <iiwa.hpp>
using namespace std;

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "cartesian_lin_drop_pose_control_node");

  cam::Kuka kuka;
  kuka.set_vel_acc_drop(0.5);

  geometry_msgs::Pose pose;
  pose.position.x = 0.48;
  pose.position.y = 0;
  pose.position.z = 0.2;
  pose.orientation.w = 0;
  pose.orientation.x = 0;
  pose.orientation.y = 1;
  pose.orientation.z = 0;

  kuka.move_cart_ptp_drop(pose);

  kuka.set_printer(true);
  kuka.set_vel_acc_lin_drop();

  cam::press_to_go();
  pose.position.y = 0;
  pose.position.z = 0.1;
  kuka.move_cart_lin_drop(pose);

  cam::press_to_go();
  pose.position.y = 0.2;
  pose.position.z = 0.2;
  kuka.move_cart_lin_drop(pose);

  cam::press_to_go();
  kuka.set_vel_acc_lin_drop(0.3);
  cam::press_to_go();
  pose.position.y = 0;
  pose.position.z = 0.2;
  kuka.move_cart_lin_drop(pose);


  ros::shutdown();

  return 0;
}
