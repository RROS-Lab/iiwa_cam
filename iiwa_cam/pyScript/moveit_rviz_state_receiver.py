#!/usr/bin/env python3
import socket
import sys
import time
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from iiwa_msgs.msg import JointPosition


class Kuka_state:
    def __init__(self,robot_name):
        self.name = robot_name
        self.topic_name =  '/' + robot_name + "/state/JointPosition"
        self.sub_hw = rospy.Subscriber(self.topic_name, JointPosition, self.joint_state)
        self.pub_state = rospy.Publisher('joint_states', JointState, queue_size = 10)
    def joint_state(self, msg):
        pos = msg.position
        jointstate = JointState()
        jointstate.header = Header()
        jointstate.header.stamp = rospy.Time.now()
        jointstate.name = [self.name + '_joint_' + str(x+1) for x in range(7) ]
        jointstate.position = [pos.a1,pos.a2,pos.a3,pos.a4,pos.a5,pos.a6,pos.a7]
        self.pub_state.publish(jointstate)


if __name__ == "__main__":

    # call the trajectory command subscriber
    rospy.init_node("kuka_publisher")
    Kuka_state("iiwa1")
    Kuka_state("iiwa2")
    rospy.spin()