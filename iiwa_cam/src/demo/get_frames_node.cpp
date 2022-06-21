/**
 * @file get_frames_node.cpp
 * @author Peijie Xu (peijiexu@usc.edu), Zhubo Zhou (zhubozho@usc.edu)
 * @brief This file shows how to move robot to pre-set points using
 * cam::Kuka::get_recorded_frames
 * @version 0.1
 * @date 2022-06-17
 *
 * @copyright Copyright (c) 2022
 *
 */
#include <iiwa.hpp>

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "get_frames_node");

  cam::Kuka kuka;

  // get recorded frames from pendant
  auto world_frame = kuka.get_recorded_frames();

  // get child frame specified by name
  auto p1 = world_frame->get_child("P1");
  auto p1_p2 = p1->get_child("P2");

  // get all children frames of P1, stored in a vector
  auto p1_children = p1->get_children();

  // kuka.move_joint_ptp(p1);

  std::cout << "moving to /P1/P2: " << std::endl
            << p1_p2->frame->get_cartesian_pos().first << std::endl
            << "status: " << p1_p2->frame->get_cartesian_pos().second
            << std::endl;

  // kuka.move_cart_ptp(p1_p2);

  ros::spin();
  ros::shutdown();
  return 0;
}
