#!/usr/bin/env python

import sys
import rospy
from xarm_planner.srv import *
from geometry_msgs.msg import Pose

import xArmPlanner_srv2python_wrapper as xArm_srv_wrapper

"""
Main script to execute that handles xArm motion for Corn Stalk insertion for CMU-ISU 2023 field test. 

Author:
- Mark Lee (MoonRobotics@cmu.edu)

Date:
- Apr 2023.
"""



if __name__ == "__main__":
    print(" ================ start ============ ")

    #obtained from manually playing with MoveIt and then reading joint postions
    STOW_JOINT_ANGLES = [-0.005691000155291448, -1.5617350979917042, -3.817223679725146e-05, -0.0002003826846523893, -0.00011472079979313321, 3.616551335650087e-05]
    APPROACH_JOINT_ANGLES = [-0.02991102667951928, -0.311088608000313, -1.189797271512119, 1.6267551770805806, 1.47070589650495, -0.0957004890775206]

    #create wrapper object
    xArm_srv_wrapper = xArm_srv_wrapper.Srv2Python()

    #move to stow position
    request_states = xArm_srv_wrapper.request_srv_joint_plan(STOW_JOINT_ANGLES)
    print(f"requesting plan: STOW_JOINT_ANGLES, Completed True/False?: {request_states}")
    request_states = xArm_srv_wrapper.request_srv_execute_plan(True)
    print(f"requesting exeuction: STOW_JOINT_ANGLES, Completed True/False?: {request_states}")

    #move to approach position
    request_states = xArm_srv_wrapper.request_srv_joint_plan(APPROACH_JOINT_ANGLES)
    print(f"requesting plan: APPROACH_JOINT_ANGLES, Completed True/False?: {request_states}")
    request_states = xArm_srv_wrapper.request_srv_execute_plan(True)
    print(f"requesting exeuction: STOW_JOINT_ANGLES, Completed True/False?: {request_states}")


    # request_srv_cartesian_plan(0.3, 0,0, 1,0,0,0)