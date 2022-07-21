////////////////////////////////////// from robot.h //////////////////////////////////////
#ifndef ROBOT_H
#define ROBOT_H

// ROS + basic libraries
#include <ros/ros.h>
#include <tf/tf.h>
#include <iostream>
#include <vector>
#include <cmath>
#include <Eigen/Eigen>

// moveit libraries
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

// moveit + ROS messages
#include <moveit_msgs/CollisionObject.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <moveit_msgs/RobotTrajectory.h>
#include "std_msgs/Bool.h"
#include "std_msgs/Header.h"
#include "std_msgs/String.h"

// #include "robot_definitions/helper.h"
#include <iiwa_cam/iiwa.hpp>

class robot{
    protected:
        ros::NodeHandle n;
        std::string robotName;
        moveit::planning_interface::MoveGroupInterfacePtr robotptr;
        std::vector<double> robotStiffenss;
        std::vector<double> robotDamping;
        bool impedanceBool;
        cam::Kuka iiwa;
        geometry_msgs::PoseStamped current_pose;

    private:
        // initializing nodehandle and movegroup parameters
        moveit::planning_interface::PlanningSceneInterface scene_interface;
        moveit::planning_interface::MoveGroupInterface::Plan myPlan;
        robot_model_loader::RobotModelLoader robot_model_loader;
        trajectory_processing::IterativeParabolicTimeParameterization iptp;
        ros::Publisher planning_scene_publisher;

    public:
        // default constructor
        robot(std::string robotName, std::string kukaName);
        robot(std::string robotName);

        // plan a path and move to a given pose
        void planMove(const geometry_msgs::Pose& goalPose, double planningTime);
        void execute();

        // plan a joint-space plan and move
        void jointPlanMove(std::vector <double> jointPos, double planningTime);

        // move the robot to the home position
        void moveHome();

        // obtain the end effector's position (x, y, z, quaternion)
        void printStates();

        // stop the robot
        void stop();

        // plan and execute a cartesian trajectory 
        void cartesian(double x_end, double y_end, double z_end, double velScale, double accScale);

        // obtain a vector of angles in radian units
        std::vector<double> getAngles(double ang1, double ang2, double ang3, double ang4, double ang5, double ang6, double ang7);
        std::vector<double> getAngles(std::vector<double> angles);

        // change the goal orientation and position tolerance
        void changeTolerance(double posTol, double oriTol);

        void changeVelScale(double scale);

        void updatePlanningScene();
};

#endif

////////////////////////////////////// from kuka.h ////////////////////////////////////// 
// #include "robotiq_gripper/change_state.h"
// #include "service_test/verify.h"
#include "iiwa_msgs/ConfigureControlMode.h"
#include "iiwa_msgs/JointQuantity.h"
#include <tf/tf.h>

class kuka : public robot{
    private:
        ros::ServiceClient client;
    public:
        kuka(std::string robotName, std::string kukaName);
        void imp_PlanMove(const geometry_msgs::Pose& goalPose, double planningTime, std::vector<double> stiffness, std::vector<double> damping, bool impedanceOrNot);
        void imp_jointPlanMove(std::vector <double> jointPos, double planningTime, std::vector<double> stiffness, std::vector<double> damping, bool impedanceOrNot);
        void imp_cartPlanMove(double x_end, double y_end, double z_end, double velScale, double accScale, std::vector<double> stiffness, std::vector<double> damping, bool impedanceOrNot);
};

////////////////////////////////////// from kuka.cpp ////////////////////////////////////// 

namespace consts{
    std::vector<double> frameBeforePickup = {4.33, 92.56, 69.75, -58.51, -1.52, 119.88, 70.97};
    double framePickupDelta = -0.05422;
    double framePlaceHeight = 0.08;
}

kuka::kuka(std::string robotName, std::string kukaName) : robot(robotName, kukaName){
    ros::ServiceClient client = n.serviceClient<iiwa_msgs::ConfigureControlMode>("/iiwa2/configuration/ConfigureControlMode");
}

void kuka::imp_PlanMove(const geometry_msgs::Pose& goalPose, double planningTime, std::vector<double> stiffness, std::vector<double> damping, bool impedanceOrNot){

        robotStiffenss = stiffness;
        robotDamping = damping;
        impedanceBool = impedanceOrNot;
        
        planMove(goalPose, planningTime);
        std::cout<<"Stiffness and Damping Set"<<std::endl;

}

void kuka::imp_jointPlanMove(std::vector <double> jointPos, double planningTime, std::vector<double> stiffness, std::vector<double> damping, bool impedanceOrNot){
    robotStiffenss = stiffness;
    robotDamping = damping;
    impedanceBool = impedanceOrNot;

    jointPlanMove(jointPos, planningTime);
    std::cout<<"Stiffness and Damping Set"<<std::endl;
}

void kuka::imp_cartPlanMove(double x_end, double y_end, double z_end, double velScale, double accScale, std::vector<double> stiffness, std::vector<double> damping, bool impedanceOrNot){
    robotStiffenss = stiffness;
    robotDamping = damping;
    impedanceBool = impedanceOrNot;

    cartesian(x_end, y_end, z_end, velScale, accScale);
    std::cout<<"Stiffness and Damping Set"<<std::endl;
}

////////////////////////////////////// from robot.cpp ////////////////////////////////////// 

namespace consts{
    // the robot's home position
    std::vector<double> home_joint_position = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
}


// default constructor
robot::robot(std::string robotName, std::string kukaName) : robot_model_loader("/iiwa/robot_description"), iptp(100, 0.5), iiwa(kukaName){
    // initialize the kuka manipulator's move group interface
    moveit::planning_interface::MoveGroupInterface::Options opt(robotName, "/iiwa/robot_description", n);
    robotptr = std::make_shared<moveit::planning_interface::MoveGroupInterface>(opt);
    this->robotName = robotName;

    // set planning parameters
    robotptr->setPlanningTime(15);
    robotptr->setNumPlanningAttempts(200);
    robotptr->setGoalPositionTolerance(0.001);
    robotptr->setGoalOrientationTolerance(0.001);

    robotptr->setMaxVelocityScalingFactor(0.2);
    robotptr->setPlannerId("AnytimePathShortening");

    planning_scene_publisher = n.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    ros::Rate loop_rate(100);
}

robot::robot(std::string robotName) : robot_model_loader("/iiwa/robot_description"), iptp(100, 0.5){
    
    // initialize the kuka manipulator's move group interface
    moveit::planning_interface::MoveGroupInterface::Options opt(robotName, "/iiwa/robot_description", n);
    robotptr = std::make_shared<moveit::planning_interface::MoveGroupInterface>(opt);
    this->robotName = robotName;

    // set planning parameters
    robotptr->setPlanningTime(15);
    robotptr->setNumPlanningAttempts(200);
    robotptr->setGoalPositionTolerance(0.001);
    robotptr->setGoalOrientationTolerance(0.001);

    robotptr->setMaxVelocityScalingFactor(0.2);
    robotptr->setPlannerId("AnytimePathShortening");

    planning_scene_publisher = n.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    ros::Rate loop_rate(100);
}

void robot::planMove(const geometry_msgs::Pose& goalPose, double planningTime){
    robotptr->setPlanningTime(planningTime);

    // a new goal message variable
    geometry_msgs::Pose goal;

    // constant goal orientation
    double quatw = goalPose.orientation.w;
    double quatx = goalPose.orientation.x;
    double quaty = goalPose.orientation.y;
    double quatz = goalPose.orientation.z;

    tf::Quaternion q(quatx, quaty, quatz, quatw);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    q.setRPY(roll, pitch, yaw);

    // assign a new position
    goal.position.x = goalPose.position.x;
    goal.position.y = goalPose.position.y;
    goal.position.z = goalPose.position.z;

    goal.orientation.w = q.getW();
    goal.orientation.x = q.getX();
    goal.orientation.y = q.getY();
    goal.orientation.z = q.getZ();

    std::cout<<goal<<std::endl;

    // set the start and target positions of a path
    robotptr->setStartStateToCurrentState();
    robotptr->setPoseTarget(goal);

    // plan a path until succeeding
    while(true){
        bool success = (robotptr->plan(myPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if(success == true){
            break;
        }
    }

    execute();
    
}

void robot::execute(){
    std::cout << robotName <<std::endl;
    std::cout << impedanceBool <<std::endl;

    if(robotName == "abb"){
        robotptr->execute(myPlan);
        ros::Duration(2.0).sleep();
    }

    else if(robotName == "kuka_green" || robotName == "kuka_blue"){
        if(impedanceBool == true){
            std::cout << "Before Impedance Control" << std::endl;
            iiwa.exe_joint_traj(myPlan.trajectory_, 0.1, robotStiffenss, robotDamping);

            std::cout << "Impedence Control Applied" << std::endl;
            std::string respone;
            std::cout<<"Done?";
            std::cin>>respone;
        }
        else{
            iiwa.exe_joint_traj(myPlan.trajectory_, 0.1, 
                        5000, 5000, 5000,
                        0.99, 0.99, 0.99);
            std::string respone;
            std::cout<<"Done?";
            std::cin>>respone;
        }
    }

    else{
        robotptr->execute(myPlan);
        std::string respone;
        std::cout<<"Done?";
        std::cin>>respone;
    }
    ROS_INFO("Trajectory execution completed.");
}

void robot::jointPlanMove(std::vector <double> jointPos, double planningTime){
    robotptr->setPlanningTime(planningTime);

    // set the start position of a path
    robotptr->setStartStateToCurrentState();
    std::cout << "Set Start State" << std::endl;

    // set the home's joint space target to the robot's home position
    robotptr->setJointValueTarget(jointPos);
    std::cout << "Set Joint Value Target!!" << std::endl;

    // plan a path until succeeding
    while(true){
        bool success = (robotptr->plan(myPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if(success == true){
            break;
        }
    }
    std::cout << "Now Executing..." << std::endl;
    execute();

}

void robot::moveHome(){
    // set the start position of a path
    robotptr->setStartStateToCurrentState();

    // set the home's joint space target to the robot's home position
    robotptr->setJointValueTarget(consts::home_joint_position);

    // plan the path
    bool success = (robotptr->plan(myPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    // if a path can be planned
    if(success == true){

        // execute the planned path
        robotptr->execute(myPlan);
    }
    std::cout<<(myPlan.trajectory_.joint_trajectory.points.back().time_from_start.nsec)*pow(10,-9)<<std::endl;
    double waitTime = (myPlan.trajectory_.joint_trajectory.points.back().time_from_start.nsec)*pow(10,-9) + 5.0;
    std::string respone;
    std::cout<<"Done?";
    std::cin>>respone;
    current_pose= robotptr ->getCurrentPose();

    ROS_INFO("Trajectory execution completed.");

}

void robot::printStates(){
    geometry_msgs::PoseStamped current_pos;
    current_pos= robotptr ->getCurrentPose();

    std::cout<<current_pos.pose.position.x<<std::endl;
    std::cout<<current_pos.pose.position.y<<std::endl;
    std::cout<<current_pos.pose.position.z<<std::endl;
    std::cout<<current_pos.pose.orientation.w<<std::endl;
    std::cout<<current_pos.pose.orientation.x<<std::endl;
    std::cout<<current_pos.pose.orientation.y<<std::endl;
    std::cout<<current_pos.pose.orientation.z<<std::endl;
    
}

// CHANGED
void robot::cartesian(double dx, double dy, double dz, double velScale, double accScale)
{
    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose target;

    const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
    robot_trajectory::RobotTrajectory someTraj(kinematic_model, this->robotName);

    geometry_msgs::PoseStamped current_pose = robotptr ->getCurrentPose();
    target= current_pose.pose;

    target.position.x = current_pose.pose.position.x + dx;
    target.position.y = current_pose.pose.position.y + dy;
    target.position.z = current_pose.pose.position.z + dz;

    waypoints.push_back(target);

    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    double eef_step = 0.01;
    double fraction = robotptr->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = "world";
    trajectory.joint_trajectory.header = header;

    if(impedanceBool == true){
        // iiwa.exe_joint_traj(trajectory, 0.1, robotStiffenss, robotDamping);
        iiwa.exe_joint_traj(trajectory, 0.02, 5000, 5000, 500, 0.7, 0.7, 0.7);
        std::cout << "Impedence Control Applied" << std::endl;
        std::string respone;
        std::cout<<"Done?";
        std::cin>>respone;
    }
    else{
        iiwa.exe_joint_traj(trajectory, 0.1, 5000, 5000, 5000, 0.99, 0.99, 0.99);
        std::string respone;
        std::cout<<"Done?";
        std::cin>>respone;
    }
    ROS_INFO("Trajectory execution completed.");
}

std::vector<double> robot::getAngles(double ang1, double ang2, double ang3, double ang4, double ang5, double ang6, double ang7){
    std::vector<double> vectortoreturn;
    vectortoreturn.push_back(ang1* 3.14159265359/180.0);
    vectortoreturn.push_back(ang2* 3.14159265359/180.0);
    vectortoreturn.push_back(ang3* 3.14159265359/180.0);
    vectortoreturn.push_back(ang4* 3.14159265359/180.0);
    vectortoreturn.push_back(ang5* 3.14159265359/180.0);
    vectortoreturn.push_back(ang6* 3.14159265359/180.0);
    if(robotName == "kuka_blue"||robotName == "kuka_green"){
        vectortoreturn.push_back(ang7* 3.14159265359/180.0);
    }

    return vectortoreturn;
}

std::vector<double> robot::getAngles(std::vector<double> angles){
    std::vector<double> anglesToReturn = angles;

    anglesToReturn[0] *= 3.14159265359/180.0;
    anglesToReturn[1] *= 3.14159265359/180.0;
    anglesToReturn[2] *= 3.14159265359/180.0;
    anglesToReturn[3] *= 3.14159265359/180.0;
    anglesToReturn[4] *= 3.14159265359/180.0;
    anglesToReturn[5] *= 3.14159265359/180.0;

    if(robotName == "kuka_blue"||robotName == "kuka_green"){
        anglesToReturn[6] *= 3.14159265359/180.0;
    }

    return anglesToReturn;
}

void robot::updatePlanningScene(){
    moveit_msgs::PlanningScene planning_scene;
    planning_scene.robot_state.is_diff = true;
    planning_scene.is_diff = true;
    scene_interface.applyPlanningScene(planning_scene);
    planning_scene_publisher.publish(planning_scene);
}

void robot::changeTolerance(double posTol, double oriTol){

    robotptr->setGoalPositionTolerance(posTol);
    robotptr->setGoalOrientationTolerance(oriTol);
}

void robot::changeVelScale(double scale){
    robotptr->setMaxVelocityScalingFactor(scale);
}

////////////////////////////////////// impedance mode ////////////////////////////////////// 
#include <moveit_msgs/ExecuteTrajectoryActionGoal.h>
using namespace std;

int main(int argc, char** argv){

    const double HEIGHT = 0.06500; // unit: m
    double X_CENTER = 0.46712; // center of our defined workspace
    double Y_CENTER = 0.00363;
    double DIST_TO_SIDE = 0.04;
    double CLOSE_ENOUGH = 0.01;
    double STEP_SIZE = 0.002; 
    double MAX_NUM_ITERS = 100;
    
    // ROS initializtion
    ros::init(argc, argv, "peg_in_hole_imp_node");
    ros::AsyncSpinner spinner(2);
    spinner.start();

    ROS_INFO_STREAM("Initializing KUKA Object");

    // kuka initializtion
    kuka okuka("manipulator", "iiwa");
    ROS_INFO_STREAM("Initialized KUKA Object");

    // Stiffness and Damping Setting for Impedance Control 
    std::vector<double> stiffness = {4000,4000,500,4000,4000,4000,4000}; 
    std::vector<double> damping = {0.7,0.7,0.7,0.7,0.7,0.7,0.7};

    // cart impedance control
    double velScale = 0.05; 
    double accScale = 0.05; 
    ROS_INFO_STREAM("Sending Move Command");
    okuka.imp_cartPlanMove(X_CENTER, Y_CENTER, HEIGHT, velScale, accScale ,stiffness, damping, true); // TODO: same stiffness definition?
    ROS_INFO_STREAM("Done sending move command");
    return 0;

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////

    // joint impedance control
    // vector<double> home = {85.32, 1.68, -1.52, 90, -1.95, 0, -5.3}; 
    // home = okuka.getAngles(home); 
    // okuka.imp_jointPlanMove(home, 5.0, stiffness, damping, true); 

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////

    // moveit::planning_interface::MoveGroupInterface *arm_ptr = NULL;

    // if (argc < 2) {
    //     // you need to run demo.lauch from iiwa_cam_moveit pkg 
    //     // before running this program
    //     arm_ptr = new moveit::planning_interface::MoveGroupInterface("iiwa_arm");
    // } 
    // else {
    //     // 1. you need to run moveit_planning_execution.launch from iiwa_moveit pkg
    //     //    before this program
    //     // 2. when you run this program, add argument:
    //     //    ns:=iiwa
    //     moveit::planning_interface::MoveGroupInterface::Options opt("manipulator", "/iiwa/robot_description");
    //     arm_ptr = new moveit::planning_interface::MoveGroupInterface(opt);
    // }
    // auto &arm = *arm_ptr;
    // cam::Kuka kuka;

    // std::string end_effector_link = arm.getEndEffectorLink();
    // std::string reference_frame = "iiwa_link_0";
    // arm.setPoseReferenceFrame(reference_frame);

    // arm.allowReplanning(true);

    // arm.setGoalPositionTolerance(0.001);
    // arm.setGoalOrientationTolerance(0.01);
    // arm.setMaxAccelerationScalingFactor(0.8);
    // arm.setMaxVelocityScalingFactor(0.8);

    // // GET CURRENT POS
    // geometry_msgs::PoseStamped current_pos;
    // current_pos= arm.getCurrentPose();

    // geometry_msgs::Pose target_pose; // TODO
    // target_pose.orientation.x = 0;
    // target_pose.orientation.y = 0;
    // target_pose.orientation.z = 1;
    // target_pose.orientation.w = 0;
    // target_pose.position.x = -0.52;
    // target_pose.position.y = 0.0;
    // target_pose.position.z = 0.15;

    // sstd::vector<geometry_msgs::Pose> waypoints;
    // waypoints.emplace_back(start_pose); // TODO
    // waypoints.emplace_back(end_pose); // TODO

    // moveit_msgs::RobotTrajectory trajectory;
    // const double jump_threshold = 0.0;
    // const double eef_step = 0.01;
    // double fraction = 0.0;
    // int maxtries = 100;
    // int attempts = 0;

    // while (fraction < 1.0 && attempts < maxtries) {
    //     fraction = arm.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    //     attempts++;
    // }

    // if (fraction < 0.5) {
    //     ROS_INFO("Path planning failed with only %0.6f success after %d attempts.", fraction, maxtries);
    // } else {
    //     if (fraction < 1.0) ROS_INFO("Path planning failed with %0.6f success after %d attempts.", fraction, maxtries); 
    //     else ROS_INFO("Path computed successfully. Moving the arm.");
    //     kuka.exe_joint_traj(trajectory);
    // }

}

