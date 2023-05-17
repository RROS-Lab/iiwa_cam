#include <string>

#include <ros/ros.h>
#include <pluginlib/class_loader.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <boost/scoped_ptr.hpp>

#include <iiwa.hpp>

const std::string iiwa_green_group = "iiwa_green_arm";
const std::string iiwa_blue_group = "iiwa_blue_arm";

// void load_planner_plugin(planning_interface::PlannerManagerPtr &planner_instance, const moveit::core::RobotModelConstPtr &robot_model, const ros::NodeHandle &node_handle)
// {
//     boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
//     std::string planner_plugin_name;

//     if (!node_handle.getParam("planning_plugin", planner_plugin_name))
//         ROS_FATAL_STREAM("Could not find planner plugin name");
//     try
//     {
//         planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
//             "moveit_core", "planning_interface::PlannerManager"));
//     }
//     catch (pluginlib::PluginlibException &ex)
//     {
//         ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
//     }
//     try
//     {
//         planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
//         if (!planner_instance->initialize(robot_model, node_handle.getNamespace()))
//             ROS_FATAL_STREAM("Could not initialize planner instance");
//         ROS_INFO_STREAM("Using planning interface '" << planner_instance->getDescription() << "'");
//     }
//     catch (pluginlib::PluginlibException &ex)
//     {
//         const std::vector<std::string> &classes = planner_plugin_loader->getDeclaredClasses();
//         std::stringstream ss;
//         for (const auto &cls : classes)
//             ss << cls << " ";
//         ROS_ERROR_STREAM("Exception while loading planner '" << planner_plugin_name << "': " << ex.what() << std::endl
//                                                              << "Available plugins: " << ss.str());
//     }
// }

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rviz_moveit_setup_node");
    ros::NodeHandle node_handle;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface iiwa_green_move_group_interface(iiwa_green_group);
    moveit::planning_interface::MoveGroupInterface iiwa_blue_move_group_interface(iiwa_blue_group);

    iiwa_green_move_group_interface.setStartStateToCurrentState();
    iiwa_blue_move_group_interface.setStartStateToCurrentState();

    // robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    // const moveit::core::RobotModelPtr &robot_model = robot_model_loader.getModel();

    // moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(robot_model));
    // const moveit::core::JointModelGroup *green_joint_model_group = robot_state->getJointModelGroup(iiwa_green_group);
    // const moveit::core::JointModelGroup *blue_joint_model_group = robot_state->getJointModelGroup(iiwa_blue_group);

    // planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));

    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("iiwa_link_0");
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

    visual_tools.loadRobotStatePub("/display_robot_state");
    visual_tools.enableBatchPublishing();
    visual_tools.deleteAllMarkers(); // clear all old markers
    visual_tools.trigger();

    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.75;
    visual_tools.publishText(text_pose, "Motion Planning API Demo", rvt::WHITE, rvt::XLARGE);

    visual_tools.trigger();

    return 0;
}