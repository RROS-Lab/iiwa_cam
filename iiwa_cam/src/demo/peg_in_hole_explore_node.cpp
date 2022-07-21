#include <iiwa.hpp>
using namespace std;
#include <random>

int main(int argc, char* argv[]) {

  ros::init(argc, argv, "peg_in_hole_explore_node");
  ros::NodeHandle nh;
  cout<<"Initializing Kuka Object\n";
  cam::Kuka kuka;

  const double HEIGHT = 0.06500; // unit: m
  double X_CENTER = 0.46712; // center of our defined workspace
  double Y_CENTER = 0.00363;
  double DIST_TO_SIDE = 0.04;
  double CLOSE_ENOUGH = 0.01;

  double STEP_SIZE = 0.002; 
  double MAX_NUM_ITERS = 100;
  
  // set velocity and accelaration for ptp
  kuka.set_vel_acc(0.02, 0.0); 
  // how to set speed for kuka.move_cart_lin()?

  // go to initial position
  cout << "press Enter to start ..." << endl;
  getchar();
  kuka.move_cart_ptp(X_CENTER, Y_CENTER, HEIGHT, 0, 0, 1, 0, 2); // unit: m
  cam::press_to_go();

  // create random device
  random_device rd;  
  mt19937 gen(rd()); 
  uniform_real_distribution<> dis(0, 2.0);

  // set limits
  double x_min=  X_CENTER - DIST_TO_SIDE;
  double x_max = X_CENTER + DIST_TO_SIDE;
  double y_min = Y_CENTER - DIST_TO_SIDE;
  double y_max = Y_CENTER + DIST_TO_SIDE;

  // pick a random start point
  double x = X_CENTER + (dis(gen) - 1) * 0.05;
  double y = Y_CENTER + (dis(gen) - 1) * 0.05;
  kuka.move_cart_ptp(x, y, HEIGHT, 0, 0, 1, 0, 2);
  
  // next, pick a series of random steps
  for (int i = 0; i < MAX_NUM_ITERS; i++) {
    
    // cout << "i = " << i << endl;
    double delta_x = (dis(gen) - 1) * STEP_SIZE;
    // cout<<"generated delta x\n";
    double delta_y = (dis(gen) - 1) * STEP_SIZE;
    // cout << "generate delta y \n";
    x = x + delta_x;

    while (x > x_max || x < x_min) {
      cout << "i = " << i << endl;
      cout<<"entered the first while"<<endl;
      // x = x - delta_x + (dis(gen) - 1) * STEP_SIZE;
      x = X_CENTER + (dis(gen) - 1) * 0.05;
    }
    y = y + delta_y;
    while (y > y_max || y < y_min) {
      cout << "i = " << i << endl;
      cout<<"entered the second while"<<endl;
      // y = y - delta_y + (dis(gen) - 1) * STEP_SIZE;
      y = Y_CENTER + (dis(gen) - 1) * 0.05;
    }
    // cout << "x = " << x << ", y = " << y << endl;
    kuka.move_cart_ptp(x, y, HEIGHT, 0, 0, 1, 0, 2);
    // cout<<"Traj executed"<<endl;
    
    if (abs(x-X_CENTER) < CLOSE_ENOUGH && abs(y-Y_CENTER) < CLOSE_ENOUGH) {
      cout << "yayyy! saved time!" << endl;
      ros::shutdown();
      return 0;
    }
  }

  cout << "mehhh, didn't find the hole within " << MAX_NUM_ITERS << " iterations" << endl;
  ros::shutdown();
  return 0;

  // TODO: sometimes the program exit with error
}
