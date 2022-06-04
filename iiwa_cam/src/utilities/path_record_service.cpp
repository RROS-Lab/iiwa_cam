#include <iiwa_cam/PathRecorder.h>
#include <ros/ros.h>

#include <cam.hpp>

void wrench_callback(const iiwa_msgs::CartesianWrench &msg) {
  geometry_msgs::WrenchStamped wrench_msg;
  wrench_msg.wrench = msg.wrench;
  wrench_msg.header.frame_id = "iiwa_link_7";
}

void cart_pos_callback(const iiwa_msgs::CartesianWrench &msg) {
  geometry_msgs::WrenchStamped wrench_msg;
  wrench_msg.wrench = msg.wrench;
  wrench_msg.header.frame_id = "iiwa_link_7";
}

class PathRecorder {
 private:
  std::string robot_name = "iiwa";
  bool recorder_state = false;

  ros::Subscriber cart_pos_sub;
  ros::Subscriber wrench_sub;

 public:
  PathRecorder() = default;
  PathRecorder(ros::NodeHandle &nh, const std::string &name) {
    robot_name = std::move(name);

    std::string robot_ns = "/";

    wrench_sub = nh.subscribe(robot_ns + robot_name + "/state/CartesianWrench",
                              10, wrench_callback);

    cart_pos_sub = nh.subscribe(
        robot_ns + robot_name + "/state/CartesianPose", 10, cart_pos_callback);
  }
};

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "path_record_service");
  ros::NodeHandle nh;

  ros::spin();
  ros::shutdown();
  return 0;
}
