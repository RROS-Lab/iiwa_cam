
#include <cam.hpp>
#include <cmath>

int main(int argc, char **argv) {
  ros::init(argc, argv, "cartesian_spline_node");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  cam::Kuka kuka;

  kuka.set_cart_traj_vel_acc(0.1, 0.1);

  std::vector<geometry_msgs::Pose> waypoints;

  if (argc > 1) {
    // read cartesian trajectory from file
    std::cout << "using data from file: " << argv[1] << std::endl;
    cam::read_traj_cart(argv[1], waypoints);

  } else {
    kuka.move_cart_ptp(-0.52, 0, 0.15, 0, 0, 1, 0);
    geometry_msgs::Pose target_pose;
    target_pose.orientation.x = 0;
    target_pose.orientation.y = 1;
    target_pose.orientation.z = 0;
    target_pose.orientation.w = 0;

    target_pose.position.x = -0.52;
    target_pose.position.y = 0.0;
    target_pose.position.z = 0.15;

    double centerA = target_pose.position.x;
    double centerB = target_pose.position.y;
    double radius = 0.1;

    for (double th = 0.0; th < 6.28; th = th + 0.01) {
      target_pose.position.x = centerA + radius * cos(th);
      target_pose.position.y = centerB + radius * sin(th);
      waypoints.emplace_back(target_pose);
    }
  }

  std::cout << "cartesian waypoints size: " << waypoints.size() << std::endl;
  kuka.exe_cart_traj(waypoints);

  return 0;
}
