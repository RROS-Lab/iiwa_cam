# Kuka ROS Tutorial - Kuka with Moveit!

[Home](../README.md)

## Content  
1. [Modified iiwa_cam_stack Moveit!](#iiwacamstack-moveit-api)
1. [Show Wrench on the Robot](#show-wrench-on-the-robot)
1. [Original iiwa_stack Moveit!](#original-iiwastack-moveit-api)


## iiwa_cam_stack Moveit API

1. Control real kuka using RVIZ:  
    in terminal, run:  
    `roslaunch iiwa_cam moveit_kuka.launch`  
    then you can trag the end effector, paln and execute.


1. Control real kuka using Cpp/Python Program:  
    **Demo File**: [moveit_plan_and_exec_node.cpp](../src/demo/moveit_plan_and_exec_node.cpp)

## Show Wrench on the Robot
  1. In terminal, run:  
    `roslaunch iiwa_cam moveit_kuka.launch`  
    Run the demo file bellow  
    **Demo File**: [wrench_rviz_state_node.cpp](../src/utilities/wrench_rviz_state_node.cpp)

  1. In RVIZ, click "Add" button -> in the "By topic" tab -> select "/iiwa/wrench" (or any topic you set in [wrench_rviz_state_node.cpp](../src/utilities/wrench_rviz_state_node.cpp))

## Original iiwa_stack Moveit API

**NOTE: This method results in a jerky motion.** [Issue link](https://github.com/IFL-CAMP/iiwa_stack/issues/284)  

1. Control real kuka:  
  in terminal, run:  
  `roslaunch iiwa_moveit moveit_planning_execution.launch sim:=false`

1. Using Gazebo to simulate:  
  in terminal, run:  
  `roslaunch iiwa_moveit moveit_planning_execution.launch`
