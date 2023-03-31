#include <actionlib/client/simple_action_client.h>
#include <iiwa_msgs/MoveToJointPositionAction.h>

#include <iiwa.hpp>
using namespace std;

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "cartesian_ptp_pose_control_node");
  ros::NodeHandle nh;

  std::string name = (argc >= 2) ? argv[1] : "iiwa";

  cam::Kuka kuka(name);

  // program start
  // phase 1: set velocity and accelaration
  kuka.set_vel_acc(0.25, 0.5);

  // phase 2: pick and put
  cam::press_to_go();
  // pick
  kuka.move_cart_ptp(-0.42, 0, 0.2, 0, 0, 1, 0);
  kuka.move_cart_ptp(0.0, 0.4, 0.2, 0, 1, 0, 0, 5);
  // put
  kuka.move_cart_ptp(0.0, 0.4, 0.1, 0, 1, 0, 0, 5);
  // going back
  kuka.move_cart_ptp(0.0, 0.4, 0.2, 0, 1, 0, 0, 5);
  kuka.move_cart_ptp(-0.42, 0, 0.2, 0, 0, 1, 0);
  kuka.move_cart_ptp(-0.42, 0, 0.1, 0, 0, 1, 0);

  ros::shutdown();

  return 0;
}
