#!/usr/bin/env python

#ROS Imports
import rospy
from geometry_msgs.msg import PoseStamped, Twist
from rospy_message_converter import message_converter as mc
import actionlib
import time


from iiwa_msgs.msg import MoveToCartesianPoseAction, MoveToJointPositionAction
from iiwa_msgs.msg import MoveAlongSplineAction
from iiwa_msgs.srv import SetPTPJointSpeedLimits
from iiwa_msgs.srv import SetSmartServoLinSpeedLimits
from iiwa_cam.srv import EndEffectorWrench
from iiwa_msgs.msg import Spline, SplineSegment
from iiwa_msgs.msg import MoveToJointPositionActionGoal



##Generic Robot Class Implementation
class Robot:
    def __init__(self, robot_name):
        self.robot_name = robot_name
        
        self.cart_state_client = rospy.ServiceProxy("/cam/"+self.robot_name+"/EndEffectorWrench", EndEffectorWrench)
        self.FORCE_WINDOW_SIZE = 30
        self.state_data = {}
    
    def get_ee_state(self, count, z_height_threshold):
        robot_state_dict = mc.convert_ros_message_to_dictionary(self.cart_state_client("iiwa"))
            
        if (robot_state_dict["pose"]["position"]["z"] < z_height_threshold - 0.002):
            robot_state_dict["label"] = 1.
        else: 
            robot_state_dict["label"] = 0.
            
        robot_state_dict["pose"] = [robot_state_dict["pose"]["position"]["x"], robot_state_dict["pose"]["position"]["y"], robot_state_dict["pose"]["position"]["z"], 
                       robot_state_dict["pose"]["orientation"]["w"], robot_state_dict["pose"]["orientation"]["x"], robot_state_dict["pose"]["orientation"]["y"], robot_state_dict["pose"]["orientation"]["z"]]

        # calculate average force data
        force_x, force_y, force_z, torque_x, torque_y, torque_z = 0., 0., 0., 0., 0., 0.
        for wrench in robot_state_dict["wrenches"]:
            force_x += wrench["force"]["x"]
            force_y += wrench["force"]["y"]
            force_z += wrench["force"]["z"]
            torque_x += wrench["torque"]["x"]
            torque_y += wrench["torque"]["y"]
            torque_z += wrench["torque"]["z"]
                
        force_x /= self.FORCE_WINDOW_SIZE
        force_y /= self.FORCE_WINDOW_SIZE
        force_z /= self.FORCE_WINDOW_SIZE
        torque_x /= self.FORCE_WINDOW_SIZE
        torque_y /= self.FORCE_WINDOW_SIZE
        torque_z /= self.FORCE_WINDOW_SIZE
                
        # update res (end effector state) 
        del robot_state_dict['success']
        del robot_state_dict['error']
        del robot_state_dict["wrenches"]
        robot_state_dict["wrench"] = [force_x, force_y, force_z, torque_x, torque_y, torque_z]
            
        # write res (end effector state) into a dict 
        self.state_data[count] = ee_cart_pose = robot_state_dict["pose"] [:3]
        # print(ee_cart_pose)
        
        return ee_cart_pose
    

#####KUKA Class#######
class Kuka(Robot):
    def __init__(self, robot_name):
        super().__init__(robot_name)
        # rospy.init_node('random_explorer_node')
        self.ptp_cart_client = actionlib.SimpleActionClient('/' + robot_name +
                                       '/action/move_to_cartesian_pose',
                                       MoveToCartesianPoseAction)

        self.ptp_joint_client = actionlib.SimpleActionClient('/' + robot_name +
                                       '/action/move_to_joint_position',
                                       MoveToJointPositionAction)

        self.lin_client = actionlib.SimpleActionClient('/' + robot_name +
                                       '/action/move_to_cartesian_pose_lin',
                                       MoveToCartesianPoseAction)

        self.lin_pub = rospy.Publisher('/' + robot_name + "/action/move_to_cartesian_pose_lin/goal", MoveToJointPositionActionGoal, queue_size = 2)

        self.joint_vel_client = rospy.ServiceProxy('/' + robot_name +
                                       '/configuration/setPTPJointLimits',
                                       SetPTPJointSpeedLimits)
        self.joint_drop_vel_client = rospy.ServiceProxy('/' + robot_name +
                                    '/configuration/setSmartServoLinLimits',
                                    SetSmartServoLinSpeedLimits)
        
        self.cart_spline_client = actionlib.SimpleActionClient('/' + robot_name +
                                        '/action/move_along_spline', MoveAlongSplineAction)

        self.lin_drop_pub = rospy.Publisher('/' + robot_name + "/command/CartesianPoseLin", PoseStamped, queue_size = 2)

    def set_vel_acc(self, vel, acc):
        if(self.joint_vel_client(vel, acc) == False):
            print("failed to change robot's joint velocity and accelaration")
    
    def set_vel_acc_lin_drop(self, vel = 0.1, acc = 0.1, override_acc = 1.0):
        twist = Twist()
        twist.linear.x = vel
        twist.linear.y = acc
        twist.linear.z = override_acc
        if(self.joint_drop_vel_client(twist) == False):
            print("failed to change robot's joint velocity and accelaration")
        

    def build_cart_act(self, posX, posY, posZ, oriW, oriX, oriY, oriZ, status , sleep_time):
        
        cart_pose_act = MoveToCartesianPoseAction()
        cart_pose_act.action_goal.goal.cartesian_pose.redundancy.status =status

        poseStamped = PoseStamped()
        poseStamped.header.frame_id = self.robot_name + "_link_0"
        poseStamped.pose.position.x = posX
        poseStamped.pose.position.y = posY
        poseStamped.pose.position.z = posZ
        poseStamped.pose.orientation.w = oriW
        poseStamped.pose.orientation.x = oriX
        poseStamped.pose.orientation.y = oriY
        poseStamped.pose.orientation.z = oriZ

        cart_pose_act.action_goal.goal.cartesian_pose.poseStamped = poseStamped

        return cart_pose_act

    def move_robot_ptp(self, goal_pose, sleep_time=600.0):
        
        print("Moving the robot to: ", goal_pose)
        self.lin_drop_pub.publish(goal_pose)
        
        time.sleep(sleep_time * 1e-3)
        return

    def move_robot_ptp_cart_action(self, goal_pose_stamped, status=2, sleep_time = 600.0):
        print("Waiting for PTP action server")
        self.ptp_cart_client.wait_for_server()
        cart_pose_act = MoveToCartesianPoseAction()
        cart_pose_act.action_goal.goal.cartesian_pose.redundancy.status = status
        cart_pose_act.action_goal.header.frame_id = self.robot_name + "_link_0"
        
        cart_pose_act.action_goal.goal.cartesian_pose.poseStamped.header = goal_pose_stamped.header
        cart_pose_act.action_goal.goal.cartesian_pose.poseStamped.pose = goal_pose_stamped.pose
        
        self.ptp_cart_client.send_goal(cart_pose_act.action_goal.goal)
        time.sleep(sleep_time * 1e-2)
        self.ptp_cart_client.wait_for_result()
        result = self.ptp_cart_client.get_result()
        print(result)
        print("Sent cartesian goal")
        return

    def move_robot_ptp_joint_action(self, joint_angles, status=2, sleep_time = 600.0):
        print("Waiting for PTP Joint Position action server")
        self.ptp_joint_client.wait_for_server()
        joint_pose = MoveToJointPositionAction()
        joint_pose.action_goal.goal.joint_position.header.frame_id = self.robot_name + "_link_0"
        joint_pose.action_goal.goal.joint_position.position.a1 = joint_angles[0]
        joint_pose.action_goal.goal.joint_position.position.a2 = joint_angles[1]
        joint_pose.action_goal.goal.joint_position.position.a3 = joint_angles[2]
        joint_pose.action_goal.goal.joint_position.position.a4 = joint_angles[3]
        joint_pose.action_goal.goal.joint_position.position.a5 = joint_angles[4]
        joint_pose.action_goal.goal.joint_position.position.a6 = joint_angles[5]
        joint_pose.action_goal.goal.joint_position.position.a7 = joint_angles[6]
        
        
        self.ptp_joint_client.send_goal(joint_pose.action_goal.goal)
        time.sleep(sleep_time * 1e-3)
        self.ptp_joint_client.wait_for_result()
        result = self.ptp_joint_client.get_result()
        print(result)
        print("Sent Joint Position Goal")
        return

    def move_robot_cart_spline(self, posestamped_array, sleep_time = 600.0):
        print("Waiting for the ros spline execution client")
        self.cart_spline_client.wait_for_server()
        # spline_traj = Spline()

        #Define the Spline segment here
        current_spline_segment = SplineSegment()
        goal = MoveAlongSplineAction()
        goal.action_goal.header.frame_id = self.robot_name + "_link_0"
        # goal.action_goal.goal.spline.segments = spline_traj.segments
        for trajectory in posestamped_array:
            current_spline_segment.type = current_spline_segment.SPL
            current_spline_segment.point.redundancy.status = 2
            current_spline_segment.point.poseStamped = trajectory
            goal.action_goal.goal.spline.segments.append(current_spline_segment)
        
        
        self.cart_spline_client.send_goal(goal.action_goal.goal)
        time.sleep(sleep_time * 1e-3)
        self.cart_spline_client.wait_for_result()
        
        result = self.cart_spline_client.get_result()
        if(result.success):
            print("Successfully completed spline segment execution")
        else:
            print("Could not execute the spline motion due to: ", result.error)

        return

    def move_cart_lin_drop(self, posX, posY, posZ, oriW, oriX, oriY, oriZ, status = 2, sleep_time = 600.0):
        
        poseStamped = PoseStamped()
        poseStamped.header.frame_id = self.robot_name+"_link_0"

        poseStamped.pose.position.x = posX
        poseStamped.pose.position.y = posY
        poseStamped.pose.position.z = posZ
        poseStamped.pose.orientation.w = oriW
        poseStamped.pose.orientation.x = oriX
        poseStamped.pose.orientation.y = oriY
        poseStamped.pose.orientation.z = oriZ
        
        self.lin_drop_pub.publish(poseStamped)
        time.sleep(sleep_time * 1e-3)

    def move_cart_ptp(self, posX, posY, posZ, oriW, oriX, oriY, oriZ, status = 2, sleep_time = 600.0):
        cart_pose_act = self.build_cart_act(posX, posY, posZ, oriW, oriX, oriY, oriZ, status , sleep_time)
        self.ptp_cart_client.send_goal(cart_pose_act.action_goal.goal)
        time.sleep(sleep_time * 1e-3)

