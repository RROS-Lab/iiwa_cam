#include <iiwa_cam/PathRecorder.h>
#include <ros/ros.h>
#include<iiwa_msgs/CartesianPose.h>
#include<iiwa_msgs/CartesianWrench.h>
// #include <iiwa.hpp>
#include <unordered_map>

class PathRecorder {
 public:
  bool recorder_state = false;

 private:
  std::string robot_name;

  ros::Subscriber cart_pos_sub;
  ros::Subscriber wrench_sub;

  void wrench_callback(const iiwa_msgs::CartesianWrench &msg) {
    if (!recorder_state)
      return;
    geometry_msgs::WrenchStamped wrench_msg;
    wrench_msg.wrench = msg.wrench;
    wrench_msg.header.frame_id = "iiwa_link_7";
  }

  void cart_pos_callback(const iiwa_msgs::CartesianPose &msg) {
    if (!recorder_state)
      return;
    msg.redundancy.e1;
    msg.poseStamped.pose.orientation.w;
  }

 public:
  PathRecorder() = default;

  PathRecorder(ros::NodeHandle &nh, const std::string &name) {
    std::cout << name << std::endl;
    robot_name = name;

    std::string robot_ns = "/";

    wrench_sub = nh.subscribe(robot_ns + robot_name + "/state/CartesianWrench",
                              10, &PathRecorder::wrench_callback, this);

    cart_pos_sub = nh.subscribe(robot_ns + robot_name + "/state/CartesianPose",
                                10, &PathRecorder::cart_pos_callback, this);
  }

  ~PathRecorder() = default;

  const std::string &get_name() const { return robot_name; }
};

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "path_record_service");
  ros::NodeHandle nh;

  std::unordered_map<std::string, PathRecorder> pr_map;

  std::cout << "Kuka Path Recorder is Listening from: " << std::endl;

  for (int i = 1; i < argc; i++) {
    std::cout << "(" << i << ") ";
    std::string name(argv[i]);

    pr_map[name] = PathRecorder(nh, name);
  }




  ros::spin();
  ros::shutdown();
  return 0;
}
