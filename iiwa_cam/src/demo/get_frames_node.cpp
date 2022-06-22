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

cam::Kuka* kuka;

// ref: http://wiki.ros.org/roscpp/Overview/Initialization%20and%20Shutdown
// we use signal handler here though it is not recommended for multithread
// programming
void sigint_handler(int sig) {
  // Do some custom action.
  // For example, publish a stop message to some other nodes.
  kuka->end_effector_state().end_recording();

  // All the default sigint handler does is call shutdown()
  ros::shutdown();
  exit(-1);
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "get_frames_node");

  kuka = new cam::Kuka();
  signal(SIGINT, sigint_handler);

  // start recording the end effector states
  kuka->end_effector_state().start_recording();

  // get recorded frames from pendant
  auto world_frame = kuka->get_recorded_frames();

  // get child frame specified by name
  auto p1 = world_frame->get_child("P1");
  if (!p1) {
    sigint_handler(2);
  }

  auto p1_p2 = p1->get_child("P2");
  if (!p1_p2) {
    sigint_handler(2);
  }

  // get all children frames of P1, stored in a vector
  auto p1_children = p1->get_children();

  kuka->move_joint_ptp(p1);

  std::cout << "moving to /P1/P2: " << std::endl
            << p1_p2->frame->get_cartesian_pos().first << std::endl
            << "status: " << p1_p2->frame->get_cartesian_pos().second
            << std::endl;

  kuka->move_cart_ptp(p1_p2);

  std::cout
      << "Waiting for motion completed ... \nPress Ctrl+C to stop recording"
      << std::endl;

  ros::spin();

  return 0;
}
