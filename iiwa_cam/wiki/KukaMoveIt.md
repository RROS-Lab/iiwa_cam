# Kuka ROS Tutorial - Kuka with Moveit!

[Home](../README.md)

## Content  
1. [Modified iiwa_cam_stack Moveit!](#moveit_cam)
1. [Original iiwa_stack Moveit!](#moveit_iiwa_stack)


<span id="moveit_iiwa_stack"></span>
## iiwa_cam_stack Moveit API

1. Control real kuka using RVIZ:  
    in terminal, run:  
    `roslaunch iiwa_cam moveit_kuka.launch`  
    then you can trag the end effector, paln and execute.


1. Control real kuka using Cpp/Python Program:  
    **Demo File**: [moveit_plan_and_exec_node.cpp](../src/demo/moveit_plan_and_exec_node.cpp)


<span id="moveit_iiwa_stack"></span>
## Original iiwa_stack Moveit API

**NOTE: This method results in a jerky motion.** [Issue link](https://github.com/IFL-CAMP/iiwa_stack/issues/284)  

1. Control real kuka:  
  in terminal, run:  
  `roslaunch iiwa_moveit moveit_planning_execution.launch sim:=false`

1. Using Gazebo to simulate:  
  in terminal, run:  
  `roslaunch iiwa_moveit moveit_planning_execution.launch`
