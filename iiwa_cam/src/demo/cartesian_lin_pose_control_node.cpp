
#include <cam.hpp>
using namespace std;

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "cartesian_lin_pose_control_node");
  ros::NodeHandle nh;

  cam::Kuka kuka;

  // program start
  // phase 1: set velocity and accelaration 
  kuka.set_vel_acc(0.1, 0.1);

  // phase 2: pick and put
  cout << "press Enter to start ..." << endl;
  getchar();
  // move to start position using ptp
  kuka.move_cart_ptp(-0.42, 0, 0.2, 0, 0, 1, 0);

  cam::press_to_go();
  kuka.move_cart_lin(-0.42, 0, 0.1, 0, 0, 1, 0);
  kuka.move_cart_lin(-0.42, 0.2, 0.2, 0, 0, 1, 0);

  // move to start position using lin
  kuka.move_cart_lin(-0.42, 0, 0.2, 0, 0, 1, 0);
  

  ros::shutdown();

  return 0;
}
