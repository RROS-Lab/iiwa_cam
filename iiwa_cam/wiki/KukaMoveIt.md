# Kuka ROS Tutorial - Kuka with Moveit!

[Home](../README.md)

## Content  
1. [Modified iiwa_stack_cam Moveit!](#iiwastackcam-moveit-api)
1. [Original iiwa_stack Moveit!](#original-iiwastack-moveit-api)


## iiwa_stack_cam Moveit API

There are two ways:

1. Control real kuka using RViz: (based on [MoveIt! - RViz Support Services](./KukaMicroservices.md#moveit---rviz-support-services))  
    in terminal, run:  
    `roslaunch iiwa_cam moveit_kuka.launch`  
    then you can trag the end effector, paln and execute.

1. Control real kuka using Cpp/Python Program:  
    **Demo File**: [moveit_plan_and_exec_node.cpp](../src/demo/moveit_plan_and_exec_node.cpp)


## Original iiwa_stack Moveit API

**NOTE: This method results in a jerky motion.** [Issue link](https://github.com/IFL-CAMP/iiwa_stack/issues/284)  

1. Control real kuka:  
  in terminal, run:  
  `roslaunch iiwa_moveit moveit_planning_execution.launch sim:=false`

1. Using Gazebo to simulate:  
  in terminal, run:  
  `roslaunch iiwa_moveit moveit_planning_execution.launch`
