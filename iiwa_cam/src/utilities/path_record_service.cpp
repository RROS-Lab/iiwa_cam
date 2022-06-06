#include <iiwa_cam/PathRecorder.h>
#include <iiwa_msgs/CartesianPose.h>
#include <iiwa_msgs/CartesianWrench.h>
#include <ros/ros.h>
// #include <iiwa.hpp>
#include <geometry_msgs/Wrench.h>

#include <mutex>
#include <unordered_map>

namespace cam {

class KukaCartesianPose {
 public:
  double cart[7];
  int status;

  KukaCartesianPose() = default;
  KukaCartesianPose(const iiwa_msgs::CartesianPose &msg) {
    cart[0] = msg.poseStamped.pose.position.x;
    cart[1] = msg.poseStamped.pose.position.y;
    cart[2] = msg.poseStamped.pose.position.z;
    cart[3] = msg.poseStamped.pose.orientation.w;
    cart[4] = msg.poseStamped.pose.orientation.x;
    cart[5] = msg.poseStamped.pose.orientation.y;
    cart[6] = msg.poseStamped.pose.orientation.z;

    status = msg.redundancy.status;
  }

  std::vector<std::string> to_vector() {
    std::vector<std::string> res;
    res.reserve(8);
    for (const auto &cart_pos : cart) {
      res.emplace_back(std::to_string(cart_pos));
    }
    res.emplace_back(std::to_string(status));
    return res;
  }
};

class KukaWrench {
 public:
  double wrench[6];

  KukaWrench() = default;
  KukaWrench(const iiwa_msgs::CartesianWrench &msg) {
    wrench[0] = msg.wrench.force.x;
    wrench[1] = msg.wrench.force.y;
    wrench[2] = msg.wrench.force.z;
    wrench[3] = msg.wrench.torque.x;
    wrench[4] = msg.wrench.torque.y;
    wrench[5] = msg.wrench.torque.z;
  }

  std::vector<std::string> to_vector() {
    std::vector<std::string> res;
    res.reserve(6);
    for (const auto &val : wrench) {
      res.emplace_back(std::to_string(val));
    }
    return res;
  }
};

class KukaRecorder {
 public:
  bool recorder_state = false;

  std::mutex *mtx;  // mutex for recorder_state

 private:
  std::string robot_name;

  ros::Subscriber cart_pos_sub;
  ros::Subscriber wrench_sub;

  std::vector<KukaCartesianPose> cart_pos_queue;
  std::vector<KukaWrench> wrench_queue;

  void wrench_callback(const iiwa_msgs::CartesianWrench &msg) {
    std::lock_guard<std::mutex> lock(*mtx);

    if (!recorder_state)
      return;

    wrench_queue.emplace_back(KukaWrench(msg));

    if (wrench_queue.size() >= 50)
      clean_wrench_queue();
  }

  void cart_pos_callback(const iiwa_msgs::CartesianPose &msg) {
    std::lock_guard<std::mutex> lock(*mtx);

    if (!recorder_state)
      return;

    cart_pos_queue.emplace_back(KukaCartesianPose(msg));

    if (cart_pos_queue.size() >= 50)
      clean_cart_pos_queue();
  }

 public:
  KukaRecorder() = default;

  KukaRecorder(ros::NodeHandle &nh, const std::string &name) {
    mtx = new std::mutex();

    cart_pos_queue.reserve(50);
    wrench_queue.reserve(50);

    std::cout << name << std::endl;
    robot_name = name;

    std::string robot_ns = "/" + robot_name;

    wrench_sub = nh.subscribe(robot_ns + "/state/CartesianWrench", 20,
                              &KukaRecorder::wrench_callback, this);

    cart_pos_sub = nh.subscribe(robot_ns + "/state/CartesianPose", 20,
                                &KukaRecorder::cart_pos_callback, this);
  }

  ~KukaRecorder() { delete mtx; }

  const std::string &get_name() const { return robot_name; }

  void clean_cart_pos_queue() {}

  void clean_wrench_queue() {}
};

}  // namespace cam

static std::unordered_map<std::string, cam::KukaRecorder *> pr_map;

bool pr_callback(iiwa_cam::PathRecorder::Request &req,
                 iiwa_cam::PathRecorder::Response &res) {
  auto pr_iter = pr_map.find(req.robot_name);
  if (pr_iter == pr_map.end()) {
    res.error = "No robot named " + req.robot_name + " is being listening to";
    res.success = false;
    return true;
  }
  auto &pr = pr_iter->second;

  std::lock_guard<std::mutex> lock(*(pr->mtx));

  if (pr->recorder_state && !req.record) {  // true -> false, do cleaning
    pr->clean_cart_pos_queue();
    pr->clean_wrench_queue();
  }
  pr->recorder_state = req.record;
  // std::cout<<pr->get_name()<<" "<<pr->recorder_state<<std::endl;

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

    pr_map[name] = new cam::KukaRecorder(nh, name);
  }

  ros::ServiceServer pr_service =
      nh.advertiseService("/cam/iiwa/PathRecorder", pr_callback);

  ros::spin();
  ros::shutdown();
  return 0;
}
