#!/usr/bin/env python3

#ROS Imports
import rospy
from geometry_msgs.msg import Twist, Pose
from rospy_message_converter import message_converter as mc
import time
import math


from iiwa_msgs.msg import MoveToCartesianPoseAction, MoveToJointPositionAction
from iiwa_msgs.msg import MoveAlongSplineAction
from iiwa_msgs.srv import SetPTPJointSpeedLimits
from iiwa_msgs.srv import SetSmartServoLinSpeedLimits

from iiwa_cam.srv import EndEffectorState
from iiwa_cam.srv import PyCartesianPose
from iiwa_cam.srv import PyCartesianSpline

from iiwa_msgs.msg import Spline, SplineSegment
from iiwa_msgs.msg import MoveToJointPositionActionGoal

def euler_from_quaternion(w, x, y, z):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    
    return roll_x, pitch_y, yaw_z # in radians

##Generic Robot Class Implementation
class Robot:
    def __init__(self, robot_name):
        self.robot_name = robot_name
        
        self.cart_state_client = rospy.ServiceProxy("/cam/"+self.robot_name+"/EndEffectorState", EndEffectorState)
        self.FORCE_WINDOW_SIZE = 10
        self.state_data = {}
    
    def get_ee_state(self, count):
        robot_state_dict = mc.convert_ros_message_to_dictionary(self.cart_state_client("iiwa"))
            
        # if (robot_state_dict["pose"]["position"]["z"] < z_height_threshold - 0.002):
        #     robot_state_dict["label"] = 1.
        # else: 
        #     robot_state_dict["label"] = 0.
        
        roll_x, pitch_y, yaw_z = euler_from_quaternion(
                                    robot_state_dict["pose"]["orientation"]["w"], 
                                    robot_state_dict["pose"]["orientation"]["x"], 
                                    robot_state_dict["pose"]["orientation"]["y"], 
                                    robot_state_dict["pose"]["orientation"]["z"])
            
        robot_state_dict["pose"] = [
            robot_state_dict["pose"]["position"]["x"], 
            robot_state_dict["pose"]["position"]["y"], 
            robot_state_dict["pose"]["position"]["z"], 
            roll_x, pitch_y, yaw_z]

        robot_state_dict["vel"] =[
            robot_state_dict["velocity"]["linear"]['x'],
            robot_state_dict["velocity"]["linear"]['y'],
            robot_state_dict["velocity"]["linear"]['z'],
            robot_state_dict["velocity"]["angular"]['x'],
            robot_state_dict["velocity"]["angular"]['y'],
            robot_state_dict["velocity"]["angular"]['z'],
        ]

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
        del robot_state_dict["velocity"]
        del robot_state_dict["wrenches"]
        
        robot_state_dict["wrench"] = [force_x, force_y, force_z, torque_x, torque_y, torque_z]
            
        # write res (end effector state) into a dict
        ee_cart_state = [robot_state_dict["stamp"]["secs"]*1000 + int(robot_state_dict["stamp"]["nsecs"] / 1000000)]
        ee_cart_state.extend(robot_state_dict["pose"])
        ee_cart_state.extend(robot_state_dict["vel"])
        ee_cart_state.extend(robot_state_dict["wrench"])
        self.state_data[count] = ee_cart_state
        # print(ee_cart_pose)
        
        return robot_state_dict
    

#####KUKA Class#######
class Kuka(Robot):
    def __init__(self, robot_name):
        super().__init__(robot_name)

        self.joint_vel_client = rospy.ServiceProxy('/' + robot_name +
                                       '/configuration/setPTPJointLimits',
                                       SetPTPJointSpeedLimits)
        self.joint_drop_vel_client = rospy.ServiceProxy('/' + robot_name +
                                    '/configuration/setSmartServoLinLimits',
                                    SetSmartServoLinSpeedLimits)

        self.cartesian_drop_client = rospy.ServiceProxy('/cam/' + robot_name + "/command/CartesianPose", PyCartesianPose)
        self.cartesian_spline_client = rospy.ServiceProxy('/cam/' + robot_name + "/command/CartesianSpline", PyCartesianSpline)

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
        

    def move_cartesian_spline(self, pose_array, status_array):
        
        cartSpline = PyCartesianSpline()
        assert len(pose_array) == len(status_array)
        cartSpline.trajectory = pose_array
        cartSpline.status = status_array
        
        self.cartesian_spline_client(cartSpline)
        
        

    def move_cartesian_drop(self, pose, linearMotion = False, sleep_time = 300.0):
        cartPose = PyCartesianPose()
        cartPose.pose = pose
        cartPose.linearMotion = linearMotion
        
        time.sleep(sleep_time * 1e-3)
        self.cartesian_drop_client(pose, linearMotion)





def main(args=None):
    rospy.init_node('test_python_robot_class')
    
    kuka = Kuka("iiwa")

    
    print(kuka.get_ee_state(1))
    kuka.get_ee_state(2)
    kuka.get_ee_state(3)
    print(kuka.state_data)



    pose = Pose()
    pose.position.x = 0.5
    pose.position.z = 0.5
    pose.orientation.y = 1

    # kuka.move_cartesian_drop(pose)
    
    poseArr = []
    statusArr = []
    r = 0.1
    theta = 0
    while theta < 2 * 3.14:
        p = Pose()
        p.position.x = 0.5 + r * math.cos(theta)
        p.position.y = r * math.sin(theta)
        p.position.z = 0.5
        p.orientation.y = 1.0
        poseArr.append(p)
        statusArr.append(2)
        theta += 0.01

    # kuka.cartesian_spline_client(poseArr, statusArr)
    # import json
    # json.dumps()
    
    rospy.spin()


if __name__ == '__main__':
    main()
    print()



