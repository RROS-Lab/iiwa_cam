
#include <iiwa.hpp>
using namespace std;

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "cartesian_lin_pose_control_node");
  ros::NodeHandle nh;

  std::string name = (argc >= 2) ? argv[1] : "iiwa";

  cam::Kuka kuka(name);

  // program start
  // phase 1: set velocity and accelaration 
  kuka.set_vel_acc(0.1, 0.1);

  // phase 2: pick and put
  cam::press_to_go();
  // move to start position using ptp
  kuka.move_cart_ptp(-0.42, 0, 0.35, 0, 0, 1, 0);

  cam::press_to_go();
  kuka.move_cart_lin(-0.42, 0, 0.3, 0, 0, 1, 0);
  kuka.move_cart_lin(-0.42, 0.2, 0.4, 0, 0, 1, 0);

  // move to start position using lin
  kuka.move_cart_lin(-0.42, 0, 0.35, 0, 0, 1, 0);
  

  ros::shutdown();

  return 0;
}
