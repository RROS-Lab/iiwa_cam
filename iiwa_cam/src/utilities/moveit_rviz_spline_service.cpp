#include <vector>

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_rviz_plugin/planning_scene_display.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <geometry_msgs/Pose.h>

#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/AttachedCollisionObject.h>

#include <iiwa.hpp>

const double jump_threshold = 0.0;
const double eef_step = 0.01;
static const std::string PLANNING_GROUP = "iiwa_arm";

static cam::Kuka *kuka;
static moveit_visual_tools::MoveItVisualTools visual_tools("iiwa_link0");
moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);

void spline_callback(const geometry_msgs::Pose &msg)
{
    move_group_interface.setPoseTarget(msg);

    moveit::planning_interface::MoveGroupInterface::Plan plan;

    if (move_group_interface.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
    {
        kuka->exe_joint_traj(plan.trajectory_);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "moveit_rviz_spline_service");
    ros::NodeHandle nh;

    static const std::string name = (argc >= 2) ? argv[1] : "iiwa";
    kuka = new cam::Kuka(name);

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    const moveit::core::RobotModelPtr &robot_model = robot_model_loader.getModel();

    moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(robot_model));
    const moveit::core::JointModelGroup *joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP);

    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();
    visual_tools.trigger();

    ros::Subscriber rviz_spline_subscriber = nh.subscribe("/" + name + "/command/RvizSpline", 100, spline_callback);

    ros::spin();
    return 0;
}