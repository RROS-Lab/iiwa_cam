#include <iiwa_cam/PathRecorder.h>
#include <iiwa_msgs/CartesianPose.h>
#include <iiwa_msgs/CartesianWrench.h>
#include <ros/ros.h>
// #include <iiwa.hpp>
#include <geometry_msgs/Wrench.h>

#include <mutex>
#include <unordered_map>

namespace cam {

class KukaRecorder {
 public:
  bool recorder_state = false;

  std::mutex *mtx;  // mutex for recorder_state

 private:
  std::string robot_name;

  ros::Subscriber cart_pos_sub;
  ros::Subscriber wrench_sub;

  void wrench_callback(const iiwa_msgs::CartesianWrench &msg) {
    // mtx.lock();
    if (!recorder_state)
      return;
    geometry_msgs::Wrench wrench_msg;
    wrench_msg = msg.wrench;
  }

  void cart_pos_callback(const iiwa_msgs::CartesianPose &msg) {
    if (!recorder_state)
      return;
    msg.redundancy.e1;
    msg.poseStamped.pose.orientation.w;
  }

 public:
  KukaRecorder() = default;

  KukaRecorder(ros::NodeHandle &nh, const std::string &name) {
    std::cout << name << std::endl;
    robot_name = name;

    std::string robot_ns = "/" + robot_name;

    wrench_sub = nh.subscribe(robot_ns + "/state/CartesianWrench", 10,
                              &KukaRecorder::wrench_callback, this);

    cart_pos_sub = nh.subscribe(robot_ns + "/state/CartesianPose", 10,
                                &KukaRecorder::cart_pos_callback, this);

    mtx = new std::mutex();
  }

  ~KukaRecorder() { delete mtx; }

  const std::string &get_name() const { return robot_name; }
};

}  // namespace cam

static std::unordered_map<std::string, cam::KukaRecorder> pr_map;

bool pr_callback(iiwa_cam::PathRecorder::Request &req,
                 iiwa_cam::PathRecorder::Response &res) {
  auto pr_iter = pr_map.find(req.robot_name);
  if (pr_iter == pr_map.end()) {
    res.error = "No robot named " + req.robot_name + " is being listening to";
    res.success = false;

    return true;
  }

  auto &pr = pr_iter->second;

  res.success = true;
  return true;
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "path_record_service");
  ros::NodeHandle nh;

  std::cout << "Kuka Path Recorder is Listening from: " << std::endl;

  for (int i = 1; i < argc; i++) {
    std::cout << "(" << i << ") ";
    std::string name(argv[i]);

    pr_map[name] = cam::KukaRecorder(nh, name);
  }

  ros::ServiceServer pr_service =
      nh.advertiseService("/cam/iiwa/PathRecorder", pr_callback);

  ros::spin();
  ros::shutdown();
  return 0;
}
