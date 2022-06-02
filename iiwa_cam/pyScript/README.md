This demo need four terminal
1. ```roscore```
2. ```rosrun iiwa_cam moveit_rviz_state_reciver.py```
3. ```rosrun iiwa_cam moveit_exec.py```
4. ```roslaunch moveitpack demo.launch``` or use moveit assistant to create package by urdf/two_iiwa.xacro 


If you want to change robot name, makesure that change the robot names coodinately in moveit_exec.py, moveit_rviz_state_receiver.py, two_iiwa.xacro and sunrise project.


