# Kuka ROS Tutorial - More About CAM::KUKA Class

[Home](../README.md)

## Content  

1. [Joint Position Control](#joint_ctrl)
1. [Cartesian Position Control](#cart_ctrl)  
   [PTP Control](#cart_ctrl_ptp)  
   [LIN Control](#cart_ctrl_lin)
1. [Joint Space Spline Trajectory](#joint_spline)  
1. [Cartesian Space Spline Trajectory](#cart_spline)  

<span id="joint_ctrl"></span>

## Joint Position Control  

**Demo File:** [joint_space_control.cpp](../other/joint_space_control.cpp)

<span id="joint_vel_ctrl"></span>

### Set joint velocity and acceleration  

We can use ROS service to control joint's velocity and acceleration of the robot  

service:  
`/iiwa/configuration/setPTPJointLimits`

message type:  
`iiwa_msgs::SetPTPJointSpeedLimits`

```cpp
  // service definition
  ros::ServiceClient joint_vel_client =
      nh.serviceClient<iiwa_msgs::SetPTPJointSpeedLimits>(joint_vel_srv_name);

  // wait for the service provided by the robot
  joint_vel_client.waitForExistence();

  // set velocity and get result
  joint_vel_msg.request.joint_relative_velocity = 0.2;
  std::cout << "set Joint Velocity to 0.2 --> " << std::flush;
  joint_vel_client.call(joint_vel_msg);
  if (joint_vel_msg.response.success)
    std::cout << "SUCCESSFUL" << std::endl;
  else
    std::cout << "FAILED" << std::endl;
```

### Set goal of joint position

There are two options to control joint position of the robot:  

- action
- command

1. **Option 1**  
    By using action:  
    `/iiwa/action/move_to_joint_position`  
    and the corresponding message type is:  
    `iiwa_msgs::MoveToJointPositionAction`  

    You need to include action library of ROS:  

    ```cpp
      #include <actionlib/client/simple_action_client.h>
    ```  

    Then define an action client object and its message:

    ```cpp
      // action definition
      actionlib::SimpleActionClient<iiwa_msgs::MoveToJointPositionAction> 
        joint_pos_client("/iiwa/action/move_to_joint_position");

      // action msg definition
      iiwa_msgs::MoveToJointPositionAction joint_pos_act;
    ```  

    Now set the target and send it to the robot

    ```cpp
      // set goal
      auto& joint_pos = joint_pos_act.action_goal.goal.joint_position;
      joint_pos.position.a1 = 0;
      joint_pos.position.a2 = 0;
      joint_pos.position.a3 = 0;
      joint_pos.position.a4 = 0;
      joint_pos.position.a5 = 0;
      joint_pos.position.a6 = 0;
      joint_pos.position.a7 = 0;

      // send message (BE CAREFUL WHEN YOU SEND FOLLOWING CODE!)
      // put your hand on the emergency button, press it when necessary
      joint_pos_client.sendGoal(joint_pos_act.action_goal.goal);
    ```

    **Note**:  
    1. If you want to send multiple goals, you need to set a gap time between these command (recommend: 500ms), for example:

        ```cpp
          // send the first goal
          joint_pos_client.sendGoal(joint_pos_act.action_goal.goal);

          ros::Duration(500 * 1e-3).sleep();

          // send the second goal
          joint_pos.position.a6 = 3.1415 / 2;
          joint_pos_client.sendGoal(joint_pos_act.action_goal.goal);
        ```

    1. The unit of joint position is radiant, for example:

        ```cpp
          // set the 6th joint to 90 degree
          joint_pos.position.a6 = 3.1415 / 2;
        ```

2. **Option 2**  
    By using topic:  
    `/iiwa/command/JointPosition`  
    and the corresponding message type is:  
    `iiwa_msgs::JointPosition`

    **Note**: In this method (through topic), the robot will abandon the previous goal immediately once it get a new goal  

3. **Other option**  
    The interface that iiwa provide is:  
    `iiwa_ros::command::JointPosition`  

    However, the robot will only execute the first goal in one program/process.
    If you want to try this method, you need to set a proper robot namespace, e.g.:

    ```cpp
      iiwa_ros::command::JointPosition cmd_jointPos;
      cmd_jointPos.init("/iiwa");

      cmd_jointPos.setPosition(joint_pos);
    ```

<span id="cart_ctrl"></span>

## Cartesian Position Control  

### Set joint velocity and acceleration

you can using joint limits to slow the velocity

<span id="cart_ctrl_ptp"></span>

### Set Cartesian position goal (Point to Point, PTP)

There are two options to control cartesian position of the robot:  

- action  (recommend)
- command

1. **Option 1**  
    By using action:  
    `/iiwa/action/move_to_cartesian_pose`  
    and the corresponding message type is:  
    `iiwa_msgs::MoveToCartesianPoseAction`  

    You need to include action library of ROS:  

    ```cpp
      #include <actionlib/client/simple_action_client.h>
    ```  

    Then define an action client object and its message:

    ```cpp
      // action definition
      actionlib::SimpleActionClient<iiwa_msgs::MoveToCartesianPoseAction> 
        cartesian_pos_client("/iiwa/action/move_to_cartesian_pose");

      // action msg definition
      iiwa_msgs::MoveToCartesianPoseAction cartesian_pos_act;
    ```  

    Now set the target and send it to the robot

    ```cpp
      // set goal
      auto & poseStamped = cartesian_pos_act.action_goal.goal.cartesian_pose.poseStamped;
      // set frame id (important!)
      poseStamped.header.frame_id = "iiwa_link_0";
      // set cartesian position
      poseStamped.pose.position.x = 0;
      poseStamped.pose.position.y = 0.4;
      poseStamped.pose.position.z = 0.3;
      poseStamped.pose.orientation.w = 0;
      poseStamped.pose.orientation.x = 0;
      poseStamped.pose.orientation.y = 1;
      poseStamped.pose.orientation.z = 0;

      // send message (BE CAREFUL WHEN YOU SEND FOLLOWING CODE!)
      // put your hand on the emergency button, press it when necessary
      cartesian_pos_client.sendGoal(cartesian_pos_act.action_goal.goal);
    ```

    **Note**:  
    1. In this method, the robot moves at a high speed. You need to use ros service to limit the speed

    1. If you want to send multiple goals, you need to set a gap time between these command (recommend: 500ms)

2. **Option 2**  
    By using topic:  
    `/iiwa/command/CartesianPose`  
    and the corresponding message type is:  
    `iiwa_msgs::CartesianPose`

    Define a topic publisher and its message:

    ```cpp
      // publisher definition
        ros::Publisher cmd_pub =
          ros::NodeHandle().advertise<iiwa_msgs::CartesianPose>(
              "/iiwa/command/CartesianPose", 10);

      // msg definition
      iiwa_msgs::CartesianPose cartesian_pos_msg;
    ```  

    Set the target and send it to the robot

    ```cpp
      // set goal
      auto & poseStamped = cartesian_pos_msg.poseStamped;

      // set frame id (important!)
      poseStamped.header.frame_id = "iiwa_link_0"

      // set cartesian position
      poseStamped.pose.position.x = 0;
      poseStamped.pose.position.y = 0.4;
      poseStamped.pose.position.z = 0.3;
      poseStamped.pose.orientation.w = 0;
      poseStamped.pose.orientation.x = 0;
      poseStamped.pose.orientation.y = 1;
      poseStamped.pose.orientation.z = 0;
      
      // send message (BE CAREFUL WHEN YOU SEND FOLLOWING CODE!)
      // put your hand on the emergency button, press it when necessary
      cmd_pub.publish(cartesian_pos_msg);

    ```

    **Note**:
    1. In this method (through topic), the robot will abaddon the previous goal immediately once it get a new goal  

<span id="cart_ctrl_lin"></span>

### Set Cartesian position goal (Linear trajectory, LIN)

There are two options to control cartesian position of the robot:  

- action (recommend)
- command

1. **Option 1**  
    By using action:  
    `/iiwa/action/move_to_cartesian_pose_lin`  
    and the corresponding message type is:  
    `iiwa_msgs::MoveToCartesianPoseAction`  

    You need to include action library of ROS:  

    ```cpp
      #include <actionlib/client/simple_action_client.h>
    ```  

    Then define an action client object and its message:

    ```cpp
      // action definition
      actionlib::SimpleActionClient<iiwa_msgs::MoveToCartesianPoseAction> 
        cartesian_pos_lin_client("/iiwa/action/move_to_cartesian_pose_lin");

      // action msg definition
      iiwa_msgs::MoveToCartesianPoseAction cartesian_pos_lin_act;
    ```  

    Now set the target and send it to the robot

    ```cpp
      // set goal
      auto & poseStamped = cartesian_pos_lin_act.action_goal.goal.cartesian_pose.poseStamped;
      // set frame id (important!)
      poseStamped.header.frame_id = "iiwa_link_0";
      // set cartesian position
      poseStamped.pose.position.x = 0;
      poseStamped.pose.position.y = 0.4;
      poseStamped.pose.position.z = 0.3;
      poseStamped.pose.orientation.w = 0;
      poseStamped.pose.orientation.x = 0;
      poseStamped.pose.orientation.y = 1;
      poseStamped.pose.orientation.z = 0;

      // send message (BE CAREFUL WHEN YOU SEND FOLLOWING CODE!)
      // put your hand on the emergency button, press it when necessary
      cartesian_pos_lin_client.sendGoal(cartesian_pos_lin_act.action_goal.goal);
    ```

    **Note**:  
    1. If you want to send multiple goals, you need to set a gap time between these command (recommend: 500ms)

2. **Option 2**  
    By using topic:  
    `/iiwa/command/CartesianPoseLin`  
    and the corresponding message type is:  
    `iiwa_msgs::CartesianPose`

    Define a topic publisher and its message:

    ```cpp
      // publisher definition
        ros::Publisher cmd_pub =
          ros::NodeHandle().advertise<iiwa_msgs::CartesianPose>(
              "/iiwa/command/CartesianPoseLin", 10);

      // msg definition
      iiwa_msgs::CartesianPose cartesian_pos_msg;
    ```  

    Set the target and send it to the robot

    ```cpp
      // set goal
      auto & poseStamped = cartesian_pos_msg.poseStamped;

      // set frame id (important!)
      poseStamped.header.frame_id = "iiwa_link_0"

      // set cartesian position
      poseStamped.pose.position.x = 0;
      poseStamped.pose.position.y = 0.4;
      poseStamped.pose.position.z = 0.3;
      poseStamped.pose.orientation.w = 0;
      poseStamped.pose.orientation.x = 0;
      poseStamped.pose.orientation.y = 1;
      poseStamped.pose.orientation.z = 0;
      
      // send message (BE CAREFUL WHEN YOU SEND FOLLOWING CODE!)
      // put your hand on the emergency button, press it when necessary
      cmd_pub.publish(cartesian_pos_msg);

    ```

    **Note**:
    1. In this method (through topic), the robot will abandon the previous goal immediately once it get a new goal  

<span id="joint_spline"></span>

## Joint Space Spline Trajectory

// TODO

<span id="cart_spline"></span>

## Cartesian Space Spline Trajectory

// TODO
