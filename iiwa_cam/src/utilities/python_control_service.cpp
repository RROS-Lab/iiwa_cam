#include <iiwa.hpp>

static cam::Kuka *kuka;

/**
 * @brief Callback function for 
 * @param req
 * @param res
 * @return true once the reqest is served
 */
bool cart_pose_ptp_drop_callback(iiwa_cam::EndEffectorState::Request &req, iiwa_cam::EndEffectorState::Response &res) {
  

  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "python_control_service");
  ros::NodeHandle nh;

  std::string name = (argc >= 2) ? argv[1] : "iiwa";

  kuka = new cam::Kuka(name);

  ros::ServiceServer cart_pose_ptp_drop_srv =
      nh.advertiseService("/cam/iiwa/command/CartesianPose", cart_pose_ptp_drop_callback);


  ros::spin();
  return 0;
}
