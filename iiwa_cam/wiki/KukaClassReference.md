# Kuka ROS Tutorial - CAM::KUKA Class API Reference

[Home](../README.md)


## Content  
- Inner Class
  - [cam::Kuka::EndEffectorState Class](#camkukaendeffectorstate-class)

- Member Functions
  - Setting Velocity, Acceleration and Jerk
    - [set_vel_acc_drop](#setvelaccdrop)
    - [set_vel_acc_lin_drop](#setvelacclindrop)
    - [set_vel_acc](#setvelacc)
    - [set_cart_traj_vel_acc](#setcarttrajvelacc)
    
  - Getting Frames from Pendant
    - [get_recorded_frames](#getrecordedframes)

  - Recording Path of the End Effector
    - [end_effector_state](#endeffectorstate)

  - Moving Manipulator to A Goal Position
    - [move_joint_ptp](#movejointptp)
    - [move_joint_ptp_drop](#movejointptpdrop)
    - [move_cart_ptp](#movecartptp)
    - [move_cart_ptp_drop](#movecartptpdrop)
    - [move_cart_lin](#movecartlin)
    - [move_cart_lin_drop](#movecartlindrop)

  - Moving Manipulator along A Trajectory
    - [exe_cart_traj](#execarttraj)
    - [exe_joint_traj](#exejointtraj)

  - Other Settings
    - [set_printer](#setprinter)

- Implementation of the Kuka Class
  - [More about Kuka Class](./KukaClassDetails.md)



## Inner Class
### cam::Kuka::EndEffectorState Class
This class works as a client for the [End Effector State Services](./KukaMicroservices.md#end-effector-state-services). If you want to record cartesian position path, wrench along the path, or get the real time cartesian position of the end effector during the runtime of a c++ program, you need to start this server before you run the c++ program. The usage is elaborated in the above link.

## Member Functions

### set_vel_acc_drop
```cpp
bool set_vel_acc_drop(const double vel = 0.1, const double acc = 0.1, 
                      const double override_acc = 1.0)  
```
Set the velocity, accelaration, override accelaration(refer its
description in the KUKA manuals) of droppable move including
[move_joint_ptp_drop](#movejointptpdrop), [move_cart_ptp_drop](#movecartptpdrop)

**Demo File:**  [cartesian_lin_drop_pose_control_node.cpp](../src/demo/cartesian_lin_drop_pose_control_node.cpp)

---   

### set_vel_acc_lin_drop

```cpp
bool set_vel_acc_lin_drop(const double vel = 0.1, const double acc = 0.1,
                          const double override_acc = 1.0)
```
Set the velocity, accelaration, override accelaration(refer its
description in the KUKA manuals) of droppable cartesian linear move:
[move_cart_lin_drop](#movecartlindrop)


**Demo File:**  [cartesian_lin_drop_pose_control_node.cpp](../src/demo/cartesian_lin_drop_pose_control_node.cpp)

---   

### set_vel_acc
```cpp
bool set_vel_acc(const double vel = 0.1, const double acc = 0.1)
```

Set the velocity and accelaration of joint space. This function
controls the velocity and accelaration of [move_joint_ptp](#movejointptp), [move_cart_ptp](#movecartptp)

**Demo File:**  [cartesian_lin_pose_control_node.cpp](../src/demo/cartesian_lin_pose_control_node.cpp)


---   

### set_cart_traj_vel_acc
```cpp
bool set_cart_traj_vel_acc(const double maxCartesianVelocity = 0.1,
                           const double maxOrientationVelocity = 0.5,
                           const double maxCartesianAcceleration = 0.2,
                           const double maxOrientationAcceleration = 0.1,
                           const double maxCartesianJerk = -1.0,
                           const double maxOrientationJerk = -1.0)
```
Set the velocity, accelaration, and jerk in cartesian space. This
function controls the velocity, accelaration, and jerk of [exe_cart_traj](#execarttraj)

**Demo File:**  [cartesian_spline_node.cpp](../src/demo/cartesian_spline_node.cpp)


---   

### get_recorded_frames
```cpp
KukaTreeNode *get_recorded_frames();
```

Get the saved frames from teaching pendant, the direct children of the world frame should be named as "P[nubmer]", e.g., "P0", "P1" ~ "P99"

**Demo File:**  [get_frames_node.cpp](../src/demo/get_frames_node.cpp)

---   

### end_effector_state
```cpp
EndEffectorState &end_effector_state()
```

Get the End Effector State object of this kuka, which allows recording path and getting the real time cartesian position



---   

### move_joint_ptp
```cpp
void move_joint_ptp(KukaTreeNode *node, const double sleep_time = 500.0);

void move_joint_ptp(const std::vector<double> &vec,
                    const double sleep_time = 500.0);


void move_joint_ptp(const double &j1, const double &j2, const double &j3,
                    const double &j4, const double &j5, const double &j6,
                    const double &j7, const double sleep_time = 500.0);
```

Move kuka point to point (PTP) assigned by joint space goal. The unit of joint position is radiant

**Demo File:**  [joint_pose_control_node.cpp](../src/demo/joint_pose_control_node.cpp)

---   

### move_joint_ptp_drop
```cpp
void move_joint_ptp_drop(const std::vector<double> &vec);

void move_joint_ptp_drop(const double &j1, const double &j2, const double &j3,
                         const double &j4, const double &j5, const double &j6,
                         const double &j7);

```
Move kuka point to point (PTP) assigned by joint space goal. The unit of joint position is radiant. The robot will abandon previous goal when it receive a new goal, even when it is still executing.


---   

### move_cart_ptp
```cpp
void move_cart_ptp(KukaTreeNode *node, const double sleep_time = 500.0);

void move_cart_ptp(const geometry_msgs::Pose &pose,
                   const int status = UNDEFINED_STATUS,
                   const double sleep_time = 500.0);

void move_cart_ptp(const double &posX, const double &posY, const double &posZ,
                   const double &oriW, const double &oriX, const double &oriY,
                   const double &oriZ, const int status = UNDEFINED_STATUS,
                   const double sleep_time = 500.0);               
```


Move kuka point to point (PTP) assigned by cartesian space goal. The unit of cartesian position is meter

**Demo File:**  [cartesian_ptp_pose_control_node.cpp](../src/demo/cartesian_ptp_pose_control_node.cpp)

---   

### move_cart_ptp_drop
```cpp
void move_cart_ptp_drop(const geometry_msgs::Pose &pose);

void move_cart_ptp_drop(const double &posX, const double &posY,
                        const double &posZ, const double &oriW,
                        const double &oriX, const double &oriY,
                        const double &oriZ);
```

Move kuka point to point (PTP) assigned by cartesian space goal. The unit of cartesian position is meter. The robot will abandon previous goal when it receive a new goal, even when it is still executing.

**Demo File:**  [cartesian_ptp_drop_pose_control_node.cpp](../src/demo/cartesian_ptp_drop_pose_control_node.cpp)



---   

### move_cart_lin
```cpp
void move_cart_lin(geometry_msgs::Pose &pose,
                   const int status = UNDEFINED_STATUS,
                   const double sleep_time = 500.0);

void move_cart_lin(const double &posX, const double &posY, const double &posZ,
                   const double &oriW, const double &oriX, const double &oriY,
                   const double &oriZ, const int status = UNDEFINED_STATUS,
                   const double sleep_time = 500.0);
```

Move kuka linearly (LIN) assigned by cartesian space goal. The unit of cartesian position is meter

**Demo File:**  [cartesian_lin_pose_control_node.cpp](../src/demo/cartesian_lin_pose_control_node.cpp)


---   

### move_cart_lin_drop
```cpp
void move_cart_lin_drop(const geometry_msgs::Pose &pose)
```

Move kuka linearly (LIN) assigned by cartesian space goal. The unit of cartesian position is meter. The robot will abandon previous goal when it receive a new goal, even when it is still executing.

**Demo File:**  [cartesian_lin_drop_pose_control_node.cpp](../src/demo/cartesian_lin_drop_pose_control_node.cpp)

---   

### exe_cart_traj
```cpp
void exe_cart_traj(const std::vector<geometry_msgs::Pose> &trajectory,
                   const std::vector<int> &status) 
```

Move robot along a trajectory in cartesian space, use set_cart_traj_vel_acc() before this method to set speed

**Demo File:**  [cartesian_spline_node.cpp](../src/demo/cartesian_spline_node.cpp)

---   

### exe_joint_traj
```cpp
void exe_joint_traj(const moveit_msgs::RobotTrajectory &trajectory,
                    const double velocity = 0.1, const double stiffX = 2000,
                    const double stiffY = 2000,  const double stiffZ = 2000)
```

Move robot along a trajectory in joint space

- velocity: joint space velocity
- stiffX, stiffY, stiffZ: stiffness on X, Y, and Z, (0.1 ~ 5000)

**Demo File:** [moveit_rviz_exec_service.cpp](../src/utilities/moveit_rviz_exec_service.cpp)

---   

### set_printer
```cpp
void set_printer(const bool &print)
```

Set the printer enabled status
- print == true: print messages

**Demo File:**  [cartesian_lin_drop_pose_control_node.cpp](../src/demo/cartesian_lin_drop_pose_control_node.cpp)

---   
