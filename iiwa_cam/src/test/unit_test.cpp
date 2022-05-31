#include <cam.hpp>

void test1() {
  cam::Frame frame(7);
  auto cart = frame.get_cartesian_pos();

  cart.push_back(1);
  auto &in = cart;
  cam::print_vec(in);
  cam::print_vec(frame.get_joint_pos());

  frame.set_joint_pos(in);

  cam::print_vec(frame.get_joint_pos());

  in.push_back(22);
  cam::print_vec(frame.get_joint_pos());
}

void test2(int argc, char *argv[]) {
  ros::init(argc, argv, "cam_unit_test2");

  cam::Kuka kuka;

  geometry_msgs::Pose pose;
  pose.position.x = -0.52;
  pose.position.y = 0;
  pose.position.z = 0.15;
  pose.orientation.w = 0;
  pose.orientation.x = 0;
  pose.orientation.y = 1;
  pose.orientation.z = 0;

  kuka.move_cart_ptp_drop(pose);

  cam::press_to_go();

  pose.position.z = 0.1;
  kuka.move_cart_ptp_drop(pose);

  cam::press_to_go();

  kuka.move_joint_ptp_drop(std::vector<double>{0, 0, 0, 0, 0, 0, 0});

  ros::spin();

  ros::shutdown();
}

void test3(int argc, char *argv[]) {
  ros::init(argc, argv, "cam_unit_test3");

  cam::Kuka kuka;
  kuka.set_vel_acc_drop(0.5);

  geometry_msgs::Pose pose;
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
}

int main(int argc, char *argv[]) {
  // test1();
  // test2(argc, argv);
  test3(argc, argv);

  return 0;
}
