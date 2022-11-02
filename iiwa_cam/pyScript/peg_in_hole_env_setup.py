#!/usr/bin/env python3

'''
This script helps 
1. set up cartesian impedance control and 
2. set the end point to "peg_point"

Some parameters to change:
1. robot_name in main()
2. stiffness_pos, stiffness_orient, damping_pos, damping_orient in setConfigMode()
3. endpoint_frame.frame_id in setEndpointFrame()

'''


import rospy
from iiwa_msgs.srv import ConfigureControlMode
from iiwa_msgs.srv import ConfigureControlModeRequest

from iiwa_msgs.srv import SetEndpointFrame
from iiwa_msgs.srv import SetEndpointFrameRequest


def setConfigMode(robot_name = "iiwa"):
  rospy.wait_for_service("/"+robot_name+"/configuration/ConfigureControlMode")
  set_control_mode = rospy.ServiceProxy("/"+robot_name+"/configuration/ConfigureControlMode", ConfigureControlMode)

  
  control_mode = ConfigureControlModeRequest()
  control_mode.control_mode = 2
  stiffness_pos = 1000
  stiffness_orient = 200
  control_mode.cartesian_impedance.cartesian_stiffness.x = stiffness_pos
  control_mode.cartesian_impedance.cartesian_stiffness.y = stiffness_pos
  control_mode.cartesian_impedance.cartesian_stiffness.z = stiffness_pos
  control_mode.cartesian_impedance.cartesian_stiffness.a = stiffness_orient
  control_mode.cartesian_impedance.cartesian_stiffness.b = stiffness_orient
  control_mode.cartesian_impedance.cartesian_stiffness.c = stiffness_orient
  
  damping_pos = 0.7
  damping_orient = 0.7
  control_mode.cartesian_impedance.cartesian_damping.x = damping_pos
  control_mode.cartesian_impedance.cartesian_damping.y = damping_pos
  control_mode.cartesian_impedance.cartesian_damping.z = damping_pos
  control_mode.cartesian_impedance.cartesian_damping.a = damping_orient
  control_mode.cartesian_impedance.cartesian_damping.b = damping_orient
  control_mode.cartesian_impedance.cartesian_damping.c = damping_orient
  
  control_mode.cartesian_impedance.nullspace_stiffness = 100.0  # should > 0
  control_mode.cartesian_impedance.nullspace_damping = 0.7
  
  print(set_control_mode(control_mode))  
  

  
def setEndpointFrame(robot_name = "iiwa"):
  rospy.wait_for_service("/"+robot_name+"/configuration/setEndpointFrame")
  set_endpoint_frame = rospy.ServiceProxy("/"+robot_name+"/configuration/setEndpointFrame", SetEndpointFrame)

  endpoint_frame = SetEndpointFrameRequest()
  endpoint_frame.frame_id = "peg_point"
  print(set_endpoint_frame(endpoint_frame))



def main(robot_name = "iiwa"):
  rospy.init_node("iiwa_env_setip")

  setEndpointFrame(robot_name)
  setConfigMode(robot_name)


if __name__== "__main__":
  robot_name = "iiwa"
  main(robot_name)  