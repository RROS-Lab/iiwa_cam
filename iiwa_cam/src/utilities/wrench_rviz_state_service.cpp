#include <cam.hpp>

static ros::Publisher wrench_pub;

void wrench_callback(const iiwa_msgs::CartesianWrench &msg) {
  geometry_msgs::WrenchStamped wrench_msg;
  wrench_msg.wrench = msg.wrench;
  wrench_msg.header.frame_id = "iiwa_link_7";

  wrench_pub.publish(wrench_msg);
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "wrench_rviz_state_service");
  ros::NodeHandle nh;
  wrench_pub = nh.advertise<geometry_msgs::WrenchStamped>("/iiwa/wrench", 10);

  ros::Subscriber sub =
      nh.subscribe("/iiwa/state/CartesianWrench", 10, wrench_callback);

  ros::spin();
  ros::shutdown();
  return 0;
}