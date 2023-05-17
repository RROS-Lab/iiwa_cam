# Kuka ROS Tutorial - Kuka with Moveit!

[Home](../README.md)

[TOC]

## iiwa_stack_cam Moveit API

There are two ways:

1. Control real kuka using RViz: (based on [MoveIt! - RViz Support Services](./KukaMicroservices.md#moveit---rviz-support-services))  
    in terminal, run:  
    `roslaunch iiwa_cam moveit_kuka.launch`  
    then you can trag the end effector, plan and execute.

  **IMPORTANT NOTE**: Check the robot name in the [source code 1 (around line 20)](../src/utilities/moveit_rviz_exec_service.cpp) and [source code 2 (around line 56)](../src/utilities/moveit_rviz_state_receiver_service.cpp)  
  
2. Control real kuka using Cpp/Python Program:  
    **Demo File**: [moveit_plan_and_exec_node.cpp](../src/demo/moveit_plan_and_exec_node.cpp)

## Original iiwa_stack Moveit API

**NOTE: This method results in a jerky motion.** [Issue link](https://github.com/IFL-CAMP/iiwa_stack/issues/284)  

1. Control real kuka:  
  in terminal, run:  
  `roslaunch iiwa_moveit moveit_planning_execution.launch sim:=false`

2. Using Gazebo to simulate:  
  in terminal, run:  
  `roslaunch iiwa_moveit moveit_planning_execution.launch`
