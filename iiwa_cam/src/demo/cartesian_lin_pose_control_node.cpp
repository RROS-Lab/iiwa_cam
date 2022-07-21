
#include <iiwa.hpp>
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
  kuka.move_cart_ptp(0.42, 0, 0.0584, 0, 0, 1, 0,2);

  cam::press_to_go();
  kuka.move_cart_lin(0.42, 0, 0.0584, 0, 0, 1, 0, 2);
  kuka.move_cart_lin(0.42, 0.2, 0.06, 0, 0, 1, 0, 2);

  // move to start position using lin
  kuka.move_cart_lin(0.42, 0, 0.0584, 0, 0, 1, 0, 2);

  // demo start
  // pick a random start point
  // pick a random x and a random y and a random direction for each
  // random x and y should be within 0 & 1
  // set limit such that if pos_x or pos_y exceed
  

  ros::shutdown();

  return 0;
}
