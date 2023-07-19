#!/usr/bin/env python3
"""
      @author   Carlos Suarez Zapico carlossuarezzapico@gmail.com
 
      @internal
      Created  7-Dec-2022
      Company  Edinburgh Centre for Robotics
      Program; Interaction Pipe with PSO optimization 
"""


# Standard imports
import sys
import rospy
import multiprocessing

sys.path.append("/home/medusa/kuka_ws/src/kmr_iiwa_carlos/scripts/kmr_iiwa/core")


from processes.IPC_processes_kuka import IPC_kuka_sharedVariables
from processes.robot_kuka_processes import robot_kuka_process
from processes.teleoperation_kuka_processes import teleop_process

# MAIN 
if __name__ == "__main__":
    myargs = rospy.myargv(argv=sys.argv)	
    IPC = IPC_kuka_sharedVariables()
    IPC.sharedWeightsVehicle[0] = 0#10000000000000000000
    IPC.sharedWeightsVehicle[1] = 0#10000000000000000000
    IPC.sharedWeightsVehicle[2] = 0#10000000000000000000
    robot = robot_kuka_process()
    teleop = teleop_process()
 
        
    #Select Processes
    robot_thread   = multiprocessing.Process(target = robot.kuka_differential_ik)
    teleop_thread  = multiprocessing.Process(target = teleop.teleop_thread)
    teleop_secondary_thread  = multiprocessing.Process(target = teleop.teleop_secondary_thread)
    
    #Start Processes
    robot_thread.start()    
    teleop_thread.start()
    teleop_secondary_thread.start()
    