#include <string>
#include <vector>
#include <unordered_map>
#include <cmath>

#include <ros/ros.h>
#include <pluginlib/class_loader.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <boost/scoped_ptr.hpp>

#include <iiwa.hpp>
#include <iiwa_cam/PickPose.h>

enum orientation
{
    bottom,
    front,
    back,
    left,
    right
};

/**
 * @brief Convert degree to radiant
 *
 * @param degree 180 degree to -180 degree
 * @return double
 */
inline double degree_to_rad(double degree)
{
    return (degree / 180.0) * M_PI;
}

// vectors used to rotate panels [w, x, y, z]
const std::vector<double> align_x_axis_vector{0.7071068, 0.7071068, 0, 0};  // 90 degree x
const std::vector<double> align_y_axis_vector{0.7071068, 0, -0.7071068, 0}; // -90 degree y

// the translation and rotation calculated from Horn's method
const std::vector<double> camera_to_robot_translation{0, 0, 0};
// [w x y z]
const std::vector<double> camera_to_robot_rotation{1, 0, 0, 0};

const std::vector<double> idea_container_size{0.4, 0.5, 0.254, 0.0127};

// std::vector<double> iiwa_blue_origin_joint{degree_to_rad(-85.72), degree_to_rad(21.7), degree_to_rad(1.59), degree_to_rad(-71.34), degree_to_rad(-93.38), degree_to_rad(50.97), degree_to_rad(-85.76)};
// std::vector<double> iiwa_blue_pre_pick_joint{degree_to_rad(-122.15), degree_to_rad(-48.86), degree_to_rad(126.21), degree_to_rad(-77.66), degree_to_rad(-65.49), degree_to_rad(49.67), degree_to_rad(-67.5)};
// std::vector<double> iiwa_blue_ready_pick_joint{degree_to_rad(-148.10), degree_to_rad(-58.63), degree_to_rad(126.21), degree_to_rad(-12.9), degree_to_rad(47.9), degree_to_rad(115.73), degree_to_rad(-24.54)};
// std::vector<double> iiwa_blue_pick_joint{degree_to_rad(-137.25), degree_to_rad(-55.8), degree_to_rad(126.22), degree_to_rad(-38.81), degree_to_rad(41.34), degree_to_rad(99.91), degree_to_rad(-32.94)};
// std::vector<double> iiwa_green_origin_joint{degree_to_rad(41.29), degree_to_rad(22.32), degree_to_rad(-60.56), degree_to_rad(-69.65), degree_to_rad(19.45), degree_to_rad(98.38), degree_to_rad(-14.3)};

static std::unordered_map<std::string, cam::Kuka *> robots;

const std::string iiwa_green_group = "iiwa_green_arm";
const std::string iiwa_blue_group = "iiwa_blue_arm";

const std::string workspace_origin_link = "iiwa_green_link_0";

const int MAX_PLAN_TIMES = 10;
const float SAFE_SPEED = 0.2;
const std::vector<float> damp(3, 0.7);
const std::vector<float> stiff(3, 1000);

moveit::planning_interface::PlanningSceneInterface *planning_scene_interface_ptr = nullptr;
moveit::planning_interface::MoveGroupInterface *iiwa_green_move_group_ptr = nullptr;
moveit::planning_interface::MoveGroupInterface *iiwa_blue_move_group_ptr = nullptr;
moveit_visual_tools::MoveItVisualTools *visual_tools_ptr = nullptr;

/**
 * @brief Convert vector of quaternion to geometry::Quaternion
 *
 * @param quat_vector [w, x, y, z]
 * @return the quaternion message
 */
geometry_msgs::Quaternion vector_to_quat(const std::vector<double> &quat_vector)
{
    geometry_msgs::Quaternion res;

    res.w = quat_vector[0];
    res.x = quat_vector[1];
    res.y = quat_vector[2];
    res.z = quat_vector[3];

    return res;
}

/**
 * @brief Convert vector of point position to geometry::Point
 *
 * @param point_vector [x, y, z]
 * @return geometry_msgs::Point
 */
geometry_msgs::Point vector_to_point(const std::vector<double> &point_vector)
{
    geometry_msgs::Point res;

    res.x = point_vector[0];
    res.y = point_vector[1];
    res.z = point_vector[2];

    return res;
}

/**
 * @brief Multiply quaternion to get rotation result
 *
 * @param a quaternion a
 * @param b quaternion b
 * @return the result quaternion
 */
geometry_msgs::Quaternion quat_mult_quat(const geometry_msgs::Quaternion &a, const geometry_msgs::Quaternion &b)
{
    geometry_msgs::Quaternion res;

    res.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
    res.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
    res.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
    res.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;

    return res;
}

/**
 * @brief Add points to get translation result
 *
 * @param a point a
 * @param b point b
 * @return the result point
 */
geometry_msgs::Point point_add_point(const geometry_msgs::Point &a, const geometry_msgs::Point &b)
{
    geometry_msgs::Point res;

    res.x = a.x + b.x;
    res.y = a.y + b.y;
    res.z = a.z + b.z;

    return res;
}

/**
 * @brief Point multiply quaternion get rotation result
 *
 * @param p point p
 * @param quat quaternion quat
 * @return the result point
 */
geometry_msgs::Point point_mult_quat(const geometry_msgs::Point &p, const geometry_msgs::Quaternion &quat)
{
    geometry_msgs::Point res;

    res.x = (1 - 2 * quat.y * quat.y - 2 * quat.z * quat.z) * p.x + (2 * quat.x * quat.y - 2 * quat.z * quat.w) * p.y + (2 * quat.x * quat.z + 2 * quat.y * quat.w) * p.z;
    res.y = (2 * quat.x * quat.y + 2 * quat.z * quat.w) * p.x + (1 - 2 * quat.x * quat.x - 2 * quat.z * quat.z) * p.y + (2 * quat.y * quat.z - 2 * quat.x * quat.w) * p.z;
    res.z = (2 * quat.x * quat.z - 2 * quat.y * quat.w) * p.x + (2 * quat.y * quat.z + 2 * quat.x * quat.w) * p.y + (1 - 2 * quat.x * quat.x - 2 * quat.y * quat.y) * p.z;

    return res;
}

/**
 * @brief Point a cross product Point b
 *
 * @param a
 * @param b
 * @return geometry_msgs::Point cross product
 */
geometry_msgs::Point point_cross_product(const geometry_msgs::Point &a, const geometry_msgs::Point &b)
{
    geometry_msgs::Point res;

    res.x = a.y * b.z - a.z * b.y;
    res.y = -(a.x * b.z - a.z * b.x);
    res.z = a.x * b.y - a.y * b.x;

    return res;
}

/**
 * @brief Give the quaternion according to the rotation matrix
 *
 * @param matrix [[x_x, x_y, x_z], [y_x, y_y, y_z], [z_x, z_y, z_z]]
 * @return geometry_msgs::Quaternion
 */
geometry_msgs::Quaternion rotation_matrix_to_quaternion(const std::vector<geometry_msgs::Point> &matrix)
{
    geometry_msgs::Quaternion res;

    double trace = matrix[0].x + matrix[1].y + matrix[2].z;
    if (trace > 1.0)
    {
        double s = 0.5 / sqrt(trace + 1.0);
        res.w = 0.25 / s;
        res.x = (matrix[1].z - matrix[2].y) * s;
        res.y = (matrix[2].x - matrix[0].z) * s;
        res.z = (matrix[0].y - matrix[1].x) * s;
    }
    else
    {
        if (matrix[0].x > matrix[1].y && matrix[0].x > matrix[2].z)
        {
            double s = 2.0 * sqrt(1 + matrix[0].x - matrix[1].y + matrix[2].z);
            res.w = (matrix[1].z - matrix[2].y) / s;
            res.x = 0.25 * s;
            res.y = (matrix[0].y + matrix[1].x) / s;
            res.z = (matrix[2].x + matrix[0].z) / s;
        }
        else if (matrix[1].y > matrix[2].z)
        {
            double s = 2.0 * sqrt(1 - matrix[0].x + matrix[1].y - matrix[2].z);
            res.w = (matrix[2].x - matrix[0].z) / s;
            res.x = (matrix[0].y + matrix[1].x) / s;
            res.y = 0.25 * s;
            res.z = (matrix[1].z + matrix[2].y) / s;
        }
        else
        {
            double s = 2.0 * sqrt(1 - matrix[0].x - matrix[1].y + matrix[2].z);
            res.w = (matrix[0].y - matrix[1].x) / s;
            res.x = (matrix[2].x + matrix[0].z) / s;
            res.y = (matrix[1].z + matrix[2].y) / s;
            res.z = 0.25 * s;
        }
    }

    return res;
}

geometry_msgs::Quaternion normalize_quaternion(const geometry_msgs::Quaternion &quat)
{
    geometry_msgs::Quaternion res;

    double sqrt_sum = sqrt(pow(quat.w, 2) + pow(quat.x, 2) + pow(quat.y, 2) + pow(quat.z, 2));
    res.w = quat.w / sqrt_sum;
    res.x = quat.x / sqrt_sum;
    res.y = quat.y / sqrt_sum;
    res.z = quat.z / sqrt_sum;

    return res;
}

geometry_msgs::Point normalize_point(const geometry_msgs::Point &p)
{
    geometry_msgs::Point res;

    double sqrt_sum = sqrt(pow(p.x, 2) + pow(p.y, 2) + pow(p.z, 2));
    res.x = p.x / sqrt_sum;
    res.y = p.y / sqrt_sum;
    res.z = p.z / sqrt_sum;

    return res;
}

/**
 * @brief Calculate the translation for each panel
 *
 * @param container_size the vector containing container shape info [x, y, z, t]
 * @param o the orientation of the panel
 * @return the translation of the panel
 */
geometry_msgs::Point container_panel_translation(const std::vector<double> &container_size, orientation o)
{
    geometry_msgs::Point res;
    res.z = container_size[2] / 2;

    switch (o)
    {
    case front:
        res.x = container_size[3] / 2 + container_size[0] / 2;
        break;
    case back:
        res.x = -container_size[3] / 2 - container_size[0] / 2;
        break;
    case left:
        res.y = -container_size[1] / 2 - container_size[3] / 2;
        break;
    case right:
        res.y = container_size[1] / 2 + container_size[3] / 2;
        break;
    }

    return res;
}

/**
 * @brief Add container to the move group interface
 *
 * @param container_center the container center
 * @param container_size the container size [x, y, z, t]
 * @return the ids of the collision objects
 */
std::vector<std::string> add_container(const geometry_msgs::Pose &container_center, const std::vector<double> &container_size)
{
    std::vector<std::string> container_objects_ids;
    std::vector<moveit_msgs::CollisionObject> container_objects;

    moveit_msgs::ObjectColor color_msg;
    color_msg.color.a = 0.5;
    color_msg.color.r = 1;
    color_msg.color.g = 1;
    color_msg.color.b = 1;
    std::vector<moveit_msgs::ObjectColor> color_msgs(5, color_msg);

    // 0: bottom, 1,2: front/back, 3/4: side left/right
    for (int i = 0; i < 5; i++)
    {
        moveit_msgs::CollisionObject panel;
        panel.id = "container_" + std::to_string(i);
        panel.header.frame_id = workspace_origin_link;
        panel.operation = moveit_msgs::CollisionObject::ADD;

        panel.primitives.resize(1);
        panel.primitives[0].type = shape_msgs::SolidPrimitive::BOX;

        panel.primitives[0].dimensions.resize(3);
        panel.primitives[0].dimensions[2] = container_size[3];
        panel.primitive_poses.resize(1);

        if (i >= 3)
        {
            panel.primitives[0].dimensions[0] = container_size[0];
            panel.primitives[0].dimensions[1] = container_size[2];

            geometry_msgs::Quaternion align_x_axis = vector_to_quat(align_x_axis_vector);

            panel.primitive_poses[0].orientation = quat_mult_quat(container_center.orientation, align_x_axis);
            panel.primitive_poses[0].position = point_add_point(container_center.position, point_mult_quat(container_panel_translation(container_size, (orientation)i), container_center.orientation));
        }
        else if (i >= 1)
        {
            panel.primitives[0].dimensions[0] = container_size[2];
            panel.primitives[0].dimensions[1] = container_size[1] + 2 * container_size[3];

            geometry_msgs::Quaternion align_y_axis = vector_to_quat(align_y_axis_vector);

            panel.primitive_poses[0].orientation = quat_mult_quat(container_center.orientation, align_y_axis);
            panel.primitive_poses[0].position = point_add_point(container_center.position, point_mult_quat(container_panel_translation(container_size, (orientation)i), container_center.orientation));
        }
        else
        {
            panel.primitives[0].dimensions[0] = container_size[0] + 2 * container_size[3];
            panel.primitives[0].dimensions[1] = container_size[1] + 2 * container_size[3];

            panel.primitive_poses[0] = container_center;
        }

        container_objects.push_back(panel);
        container_objects_ids.push_back(panel.id);
    }

    planning_scene_interface_ptr->applyCollisionObjects(container_objects, color_msgs);

    return container_objects_ids;
}

/**
 * @brief Add floor to the move group interface
 *
 * @return the ids of the collision floor
 */
std::vector<std::string> add_floor()
{
    std::vector<std::string> floor_objects_ids;
    std::vector<moveit_msgs::CollisionObject> floor_objects;

    moveit_msgs::ObjectColor color_msg;
    color_msg.color.a = 1;
    color_msg.color.r = 0.5;
    color_msg.color.g = 0.5;
    color_msg.color.b = 0.5;

    moveit_msgs::CollisionObject floor;
    floor.id = "floor";
    floor.header.frame_id = workspace_origin_link;
    floor.operation = moveit_msgs::CollisionObject::ADD;

    floor.primitives.resize(1);
    floor.primitives[0].type = shape_msgs::SolidPrimitive::BOX;

    floor.primitives[0].dimensions.resize(3);

    floor.primitives[0].dimensions[0] = 5;
    floor.primitives[0].dimensions[1] = 5;
    floor.primitives[0].dimensions[2] = 0.01;

    floor.primitive_poses.resize(1);
    floor.primitive_poses[0].position.z = -0.01;

    floor_objects.push_back(floor);
    floor_objects_ids.push_back(floor.id);

    planning_scene_interface_ptr->applyCollisionObjects(floor_objects, std::vector<moveit_msgs::ObjectColor>{color_msg});

    return floor_objects_ids;
}

void add_joint_constraints(const std::string &robot_name, moveit_msgs::Constraints &constraints)
{
    for (int i = 1; i <= 7; i++)
    {
        moveit_msgs::JointConstraint jcm;

        jcm.joint_name = robot_name + "_joint_" + std::to_string(i);
        jcm.position = 0;
        if (i < 7)
        {
            if (i % 2 == 1)
            {
                jcm.tolerance_above = M_PI * 168 / 180.0;
                jcm.tolerance_below = M_PI * 168 / 180.0;
            }
            else
            {
                jcm.tolerance_above = M_PI * 118 / 180.0;
                jcm.tolerance_below = M_PI * 118 / 180.0;
            }
        }
        else
        {
            jcm.tolerance_above = M_PI * 173 / 180.0;
            jcm.tolerance_below = M_PI * 173 / 180.0;
        }
        jcm.weight = 0.5;

        constraints.joint_constraints.push_back(jcm);
    }
}

/**
 * @brief Add path constraints for the given move group interface
 *
 * @param move_group_interface the move group interface with constraints
 * @param robot_move_group the robot move group name
 * @param quat the constraint orientation
 */
void add_orientation_constraints(moveit::planning_interface::MoveGroupInterface &move_group_interface, const std::string &robot_move_group, const geometry_msgs::Quaternion quat = vector_to_quat(std::vector<double>{0, 0, 1, 0}))
{
    std::string robot_name = robot_move_group.substr(0, robot_move_group.size() - 4);

    moveit_msgs::Constraints constraints;

    moveit_msgs::OrientationConstraint ocm;
    ocm.link_name = robot_name + "_link_7";
    ocm.header.frame_id = robot_name + "_link_0";

    ocm.orientation = quat;

    ocm.absolute_x_axis_tolerance = 0.1;
    ocm.absolute_y_axis_tolerance = 0.1;
    ocm.absolute_z_axis_tolerance = 0.1;

    ocm.weight = 1.0;

    constraints.orientation_constraints.push_back(ocm);
    // add_joint_constraints(robot_name, constraints);

    move_group_interface.setPathConstraints(constraints);
}

/**
 * @brief Remove path constraints for the given move group interface
 *
 * @param move_group_interface the move group interface with constraints
 *
 */
void delete_constrains(moveit::planning_interface::MoveGroupInterface &move_group_interface)
{
    moveit_msgs::Constraints constraints;
    move_group_interface.setPathConstraints(constraints);
}

bool plan_to_target_pose(moveit::planning_interface::MoveGroupInterface &move_group_interface, moveit_visual_tools::MoveItVisualTools &visual_tools, const std::string &robot_move_group,
                         const std::vector<double> &position, const std::vector<double> &orientation, const float velocity = SAFE_SPEED)
{

    std::string robot_name = robot_move_group.substr(0, robot_move_group.size() - 4);

    geometry_msgs::Pose target_pose;

    target_pose.position = vector_to_point(position);
    target_pose.orientation = vector_to_quat(orientation);

    move_group_interface.setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_interface.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    for (int i = 0; i < MAX_PLAN_TIMES && !success; i++)
    {
        success = (move_group_interface.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        ROS_INFO("Plan to the target pose %s", success ? "SUCCESS" : "FAILED");
    }

    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to execute the demo");

    if (success)
    {
        robots[robot_name]->exe_joint_traj(plan.trajectory_.joint_trajectory.points, velocity, stiff, damp, cam::Kuka::JOINT_SPLINE_MODE::CARTESIAN_IMPEDANCE_MODE);
    }

    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

    return success;
}

bool plan_to_target_pick_pose(iiwa_cam::PickPose::Request &req, iiwa_cam::PickPose::Response &res)
{
    geometry_msgs::Point pkg_center_position = req.pkg_center_position;
    geometry_msgs::Point pkg_center_z_direction = req.pkg_center_normal;
    geometry_msgs::Point pkg_center_x_direction = req.pkg_side_direction;

    pkg_center_position = point_add_point(point_mult_quat(pkg_center_position, vector_to_quat(camera_to_robot_rotation)), vector_to_point(camera_to_robot_translation));
    pkg_center_z_direction = normalize_point(point_mult_quat(pkg_center_z_direction, vector_to_quat(camera_to_robot_rotation)));
    pkg_center_x_direction = normalize_point(point_mult_quat(pkg_center_x_direction, vector_to_quat(camera_to_robot_rotation)));

    geometry_msgs::Point pkg_center_y_direction = point_cross_product(pkg_center_z_direction, pkg_center_x_direction);
    // normalize
    geometry_msgs::Quaternion pkg_pick_orientation = rotation_matrix_to_quaternion(std::vector<geometry_msgs::Point>{pkg_center_x_direction, pkg_center_y_direction, pkg_center_z_direction});
    geometry_msgs::Pose target_pose;
    target_pose.position = pkg_center_position;
    target_pose.orientation = normalize_quaternion(quat_mult_quat(pkg_pick_orientation, vector_to_quat(align_y_axis_vector)));

    // ROS_INFO("%f %f %f", target_pose.position.x, target_pose.position.y, target_pose.position.z);
    // ROS_INFO("%f %f %f %f", target_pose.orientation.w, target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z);

    iiwa_blue_move_group_ptr->setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (iiwa_blue_move_group_ptr->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO("Plan to the target pose %s", success ? "SUCCESS" : "FAILED");
    if (success)
    {
        visual_tools_ptr->prompt("Press 'next' in the RvizVisualToolsGui window to execute the trajectory");
        robots["iiwa_blue"]->exe_joint_traj(plan.trajectory_.joint_trajectory.points, SAFE_SPEED, stiff, damp, cam::Kuka::JOINT_SPLINE_MODE::CARTESIAN_IMPEDANCE_MODE);
    }

    res.success = success;
    if (!success)
        res.error_msg = "Failed to plan to the target position";

    return success;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rviz_moveit_setup_node");
    ros::NodeHandle node_handle;

    ros::AsyncSpinner spinner(2);
    spinner.start();

    for (int i = 1; i < argc; i++)
        robots.insert(std::pair<std::string, cam::Kuka *>(argv[i], new cam::Kuka(argv[i])));

    // robots["iiwa_blue"]->move_joint_ptp(iiwa_blue_origin_joint);
    // robots["iiwa_green"]->move_joint_ptp(iiwa_green_origin_joint);

    planning_scene_interface_ptr = new moveit::planning_interface::PlanningSceneInterface();
    iiwa_green_move_group_ptr = new moveit::planning_interface::MoveGroupInterface(iiwa_green_group);
    iiwa_blue_move_group_ptr = new moveit::planning_interface::MoveGroupInterface(iiwa_blue_group);

    iiwa_green_move_group_ptr->setStartStateToCurrentState();
    iiwa_blue_move_group_ptr->setStartStateToCurrentState();

    visual_tools_ptr = new moveit_visual_tools::MoveItVisualTools(workspace_origin_link);

    visual_tools_ptr->enableBatchPublishing();
    visual_tools_ptr->deleteAllMarkers();
    visual_tools_ptr->trigger();

    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.75;
    visual_tools_ptr->publishText(text_pose, "Motion Planning Demo", rviz_visual_tools::WHITE, rviz_visual_tools::XXLARGE);
    visual_tools_ptr->trigger();

    geometry_msgs::Pose container_center;
    container_center.orientation.w = 1;
    container_center.position.x = 0.5;
    container_center.position.y = 0.5;

    auto container_id = add_container(container_center, idea_container_size);
    auto floor_id = add_floor();

    /**
     * @brief Position setup
     *
     *
     * std::vector<double> iiwa_blue_origin_position{-0.5, 0.5, 0.7};

     * std::vector<double> iiwa_blue_before_pick_position{-0.5, 0.5, 0.2};
     * std::vector<double> iiwa_blue_pick_position{-0.5, 0.5, 0.13};

     * std::vector<double> iiwa_blue_before_place_position{0.55, 0.7, 0.5};
     * std::vector<double> iiwa_blue_place_position{0.55, 0.6, 0.5};
     * std::vector<double> iiwa_blue_place_down_position{0.55, 0.6, 0.4};

     * std::vector<double> iiwa_blue_hold_position{0.55, 0.65, 0.4};
     * std::vector<double> iiwa_blue_hold_right_position{0.55, 0.68, 0.5};
     * std::vector<double> iiwa_blue_hold_right_down_position{0.55, 0.68, 0.4};

     * std::vector<double> iiwa_blue_pick_orientation{0.7071068, 0, -0.7071068, 0};
     * std::vector<double> iiwa_blue_place_orientation{0.5, 0.5, -0.5, 0.5};
     * std::vector<double> iiwa_blue_place_down_orientation{-0.4304593, -0.5609855, 0.5609855, -0.4304593};
     * std::vector<double> iiwa_blue_hold_orientation{0, 0, 1, 0};

     * std::vector<double> iiwa_green_finished_position{0.5, 0, 0.65};

     * std::vector<double> iiwa_green_before_box_position{0.5, 0.35, 0.65};
     * std::vector<double> iiwa_green_box_start_position{0.5, 0.35, 0.35};
     * std::vector<double> iiwa_green_box_end_position{0.5, 0.5, 0.35};
     * std::vector<double> iiwa_green_pose_orientation{0, 0, 1, 0};
     */

    /**
     * @brief iiwa_green: place and go back
     *
     * plan_to_target_pose(iiwa_green_move_group_interface, visual_tools, iiwa_green_group, iiwa_green_finished_position, iiwa_green_pose_orientation);
     *
     * add_orientation_constraints(iiwa_green_move_group_interface, iiwa_green_group);
     * plan_to_target_pose(iiwa_green_move_group_interface, visual_tools, iiwa_green_group, iiwa_green_before_box_position, iiwa_green_pose_orientation);
     * plan_to_target_pose(iiwa_green_move_group_interface, visual_tools, iiwa_green_group, iiwa_green_box_start_position, iiwa_green_pose_orientation);
     * plan_to_target_pose(iiwa_green_move_group_interface, visual_tools, iiwa_green_group, iiwa_green_box_end_position, iiwa_green_pose_orientation);
     * plan_to_target_pose(iiwa_green_move_group_interface, visual_tools, iiwa_green_group, iiwa_green_before_box_position, iiwa_green_pose_orientation);
     * delete_constrains(iiwa_green_move_group_interface);
     *
     * plan_to_target_pose(iiwa_green_move_group_interface, visual_tools, iiwa_green_group, iiwa_green_finished_position, iiwa_green_pose_orientation);
     */

    /**
     * @brief iiwa_blue: pick and place
     *
     * plan_to_target_pose(iiwa_blue_move_group_interface, visual_tools, iiwa_blue_group, iiwa_blue_origin_position, iiwa_blue_pick_orientation);
     * //pick package up
     * plan_to_target_pose(iiwa_blue_move_group_interface, visual_tools, iiwa_blue_group, iiwa_blue_before_pick_position, iiwa_blue_pick_orientation);
     * plan_to_target_pose(iiwa_blue_move_group_interface, visual_tools, iiwa_blue_group, iiwa_blue_pick_position, iiwa_blue_pick_orientation, 0.05);
     * plan_to_target_pose(iiwa_blue_move_group_interface, visual_tools, iiwa_blue_group, iiwa_blue_origin_position, iiwa_blue_pick_orientation);
     *
     * //place package inside container
     * plan_to_target_pose(iiwa_blue_move_group_interface, visual_tools, iiwa_blue_group, iiwa_blue_before_place_position, iiwa_blue_place_orientation);
     * plan_to_target_pose(iiwa_blue_move_group_interface, visual_tools, iiwa_blue_group, iiwa_blue_place_position, iiwa_blue_place_orientation);
     * plan_to_target_pose(iiwa_blue_move_group_interface, visual_tools, iiwa_blue_group, iiwa_blue_place_down_position, iiwa_blue_place_down_orientation);
     * plan_to_target_pose(iiwa_blue_move_group_interface, visual_tools, iiwa_blue_group, iiwa_blue_place_position, iiwa_blue_place_orientation);
     */

    /**
     * @brief iiwa_blue: gripper pick and go back
     * grasp package using gripper
     * robots["iiwa_blue"]->move_joint_ptp(iiwa_blue_pre_pick_joint);
     * visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
     * robots["iiwa_blue"]->move_joint_ptp(iiwa_blue_ready_pick_joint);
     * visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
     * robots["iiwa_blue"]->move_joint_ptp(iiwa_blue_pick_joint);
     * visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

     * plan_to_target_pose(iiwa_blue_move_group_interface, visual_tools, iiwa_blue_group, iiwa_blue_place_position, iiwa_blue_hold_orientation);
     * plan_to_target_pose(iiwa_blue_move_group_interface, visual_tools, iiwa_blue_group, iiwa_blue_place_down_position, iiwa_blue_hold_orientation);
     * plan_to_target_pose(iiwa_blue_move_group_interface, visual_tools, iiwa_blue_group, iiwa_blue_place_position, iiwa_blue_hold_orientation);

     * plan_to_target_pose(iiwa_blue_move_group_interface, visual_tools, iiwa_blue_group, iiwa_blue_before_place_position, iiwa_blue_place_orientation);
     * robots["iiwa_blue"]->move_joint_ptp(iiwa_blue_origin_joint);
     */

    visual_tools_ptr->prompt("Press 'next' in the RvizVisualToolsGui window to end the demo");

    ros::ServiceServer pick_service = node_handle.advertiseService("pick_pose", plan_to_target_pick_pose);

    ros::waitForShutdown();
    delete (planning_scene_interface_ptr);
    delete (iiwa_green_move_group_ptr);
    delete (iiwa_blue_move_group_ptr);
    return 0;
}