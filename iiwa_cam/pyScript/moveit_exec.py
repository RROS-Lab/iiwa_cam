#!/usr/bin/env python3

import rospy
from moveit_msgs.msg import ExecuteTrajectoryActionGoal
from iiwa_msgs.msg import Spline
from iiwa_msgs.msg import SplineSegment
from iiwa_msgs.msg import MoveToJointPositionAction
import actionlib






class Kuka_JointSpline:
    def __init__(self,robot_name):
        self.name = robot_name
        self.control_topic_name =  '/' + self.name + "/command/JointSpline"
        self.client_topic_name =  '/' + self.name + "/action/move_to_joint_position"
        self.pub_hw = rospy.Publisher(self.control_topic_name, Spline, queue_size = 10)
        self.joint_pos_client = actionlib.SimpleActionClient(self.client_topic_name,MoveToJointPositionAction)

        self.sub_goal = rospy.Subscriber('/execute_trajectory/goal', ExecuteTrajectoryActionGoal,self.receive_goal )

    def receive_goal(self, msg):
        points = msg.goal.trajectory.joint_trajectory.points
        joint_name = msg.goal.trajectory.joint_trajectory.joint_names[0]
        rate = rate = rospy.Rate(2) # 500ms

        # if a trajectory command is received 
        if(len(points)!=0):
            if(joint_name == self.name + "_joint_1"):
                print(self.name + "excute spline")
            else:
                print(self.name, "do nothing")
                return

            spline = Spline()
            for x in points:
                seg = SplineSegment()
                seg.point.poseStamped.pose.position.x = x.positions[0]
                seg.point.poseStamped.pose.position.y = x.positions[1]
                seg.point.poseStamped.pose.position.z = x.positions[2]
                seg.point.poseStamped.pose.orientation.w = x.positions[3]
                seg.point.poseStamped.pose.orientation.x = x.positions[4]
                seg.point.poseStamped.pose.orientation.y = x.positions[5]
                seg.point.poseStamped.pose.orientation.z = x.positions[6]
                spline.segments.append(seg)

            action = MoveToJointPositionAction()
            action.action_goal.goal.joint_position.position.a1 = points[0].positions[0]
            action.action_goal.goal.joint_position.position.a2 = points[0].positions[1]
            action.action_goal.goal.joint_position.position.a3 = points[0].positions[2]
            action.action_goal.goal.joint_position.position.a4 = points[0].positions[3]
            action.action_goal.goal.joint_position.position.a5 = points[0].positions[4]
            action.action_goal.goal.joint_position.position.a6 = points[0].positions[5]
            action.action_goal.goal.joint_position.position.a7 = points[0].positions[6]
            self.joint_pos_client.send_goal(action.action_goal.goal)
            rate.sleep()
            self.pub_hw.publish(spline)
        

if __name__ == "__main__":
    # call the trajectory command subscriber
    rospy.init_node("Kuka_JointSpline")
    Kuka_JointSpline("iiwa1")
    Kuka_JointSpline("iiwa2")
    rospy.spin()