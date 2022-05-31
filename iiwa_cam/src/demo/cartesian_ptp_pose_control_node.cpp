#include <actionlib/client/simple_action_client.h>
#include <iiwa_msgs/MoveToJointPositionAction.h>

#include <cam.hpp>
using namespace std;

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "cartesian_ptp_pose_control_node");
  ros::NodeHandle nh;

  cam::Kuka kuka;

  // program start
  // phase 1: set velocity and accelaration 
  kuka.set_vel_acc(0.25, 0.5);

  // phase 2: pick and put
  cout << "press Enter to continue ..." << endl;
  getchar();
  // pick
  kuka.move_cart_ptp(-0.42, 0, 0.2, 0, 0, 1, 0);
  kuka.move_cart_ptp(0.0, 0.4, 0.2, 0, 1, 0, 0);
  // put
  kuka.move_cart_ptp(0.0, 0.4, 0.1, 0, 1, 0, 0);
  // going back
  kuka.move_cart_ptp(0.0, 0.4, 0.2, 0, 1, 0, 0);
  kuka.move_cart_ptp(-0.42, 0, 0.2, 0, 0, 1, 0);
  kuka.move_cart_ptp(-0.42, 0, 0.1, 0, 0, 1, 0);

  ros::shutdown();

  return 0;
}
