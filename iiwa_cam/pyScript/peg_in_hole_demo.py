#!/usr/bin/env python

import rospy
from iiwa_cam.srv import EndEffectorWrench
import csv

def end_effector_state_recorder():
    
    rospy.wait_for_service('/cam/iiwa/EndEffectorWrench')

    try:
        get_ee = rospy.ServiceProxy('/cam/iiwa/EndEffectorWrench', EndEffectorWrench) 
        ee = get_ee('iiwa')
        return (ee.pose, ee.wrenches)

    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        

if __name__ == "__main__":

    while (True):
    # for t in range(10000): # exit safely? 
        rospy.init_node('peg_in_hole_demo')
        ee = end_effector_state_recorder()
        # print("pose: ", ee[0])
        print("wrench: \n")
        print(ee[1])

        # set rate
        rate = rospy.Rate(2) # 2hz
        rate.sleep()

        # save to csv
        # f = open('../data', 'data')
        # writer = csv.writer(f)
        # writer.writerow(row)

    # f.close()
        
    # more accurate rate control: import threading?
    