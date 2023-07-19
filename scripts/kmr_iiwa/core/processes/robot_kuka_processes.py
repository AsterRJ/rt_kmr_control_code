#!/usr/bin/env python3
"""
      @author   Carlos Suarez Zapico carlossuarezzapico@gmail.com
 
      @internal
      Created  22-Dec-2022
      Company  Edinburgh Centre for Robotics
      Class Definition; Inter Process Comunication Class Extension -- ROBOT IO METHODS
"""

import os
import rospy
import numpy as np
import time
from processes.IPC_processes_kuka import IPC_kuka_sharedVariables
from libraries.aux_tools import aux_tools
from libraries.kmr_iiwa import kmr_iiwa

class robot_kuka_process(IPC_kuka_sharedVariables):

    def kuka_differential_ik(self): 
        rospy.init_node("robot_motion", anonymous=True)
        robot = kmr_iiwa() 
        aux_tools_instance = aux_tools()
        joint_values      = np.array([0.0, 0.0, 0.0, 1.7, 1.0, 0.1, -1.6, -0.2, -1.2, 0.0])
        q_output_velocity = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        task_space_vel    = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        secundary_goal    = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        sampling_control = 0.0001
        time_last_action = time.time()
        while (not rospy.is_shutdown()):  
            if ((time.time()- time_last_action)>sampling_control):
                time_last_action = time.time()       
                #OUTPUT TO SHARED VARIABLES
                aux_tools_instance.copy_sharedArray_to_npArray(self.sharedTaskVel, task_space_vel)
                aux_tools_instance.copy_sharedArray_to_npArray(self.sharedSecondaryVel, secundary_goal)
                #PENALTIES
                robot.penaltyX = self.sharedWeightsVehicle[0]
                robot.penaltyY = self.sharedWeightsVehicle[1]
                robot.penaltyYaw = self.sharedWeightsVehicle[2]     
                #INTEGRATION
                joint_values
                q_output = robot.weighted_base_robust_mobile_axis(joint_values[0] ,joint_values[1], joint_values[2], joint_values[3:10], task_space_vel, secundary_goal, robot.penaltyX, robot.penaltyY, robot.penaltyYaw)
                #q_output = robot.redundancy_robust_mobile_axis(joint_values[0] ,joint_values[1], joint_values[2], joint_values[3:10], task_space_vel, secundary_goal)     
                q_output_velocity = np.array([q_output[0,0], q_output[0,1], q_output[0,2], q_output[0,3], q_output[0,4], q_output[0,5], q_output[0,6], q_output[0,7], q_output[0,8], q_output[0,9]])
                joint_values = joint_values + q_output_velocity*sampling_control   
                robot.publish_joint_states(joint_values)             
                 
