#!/usr/bin/env python3
"""
      @author   Carlos Suarez Zapico carlossuarezzapico@gmail.com
 
      @internal
      Created  22-Dec-2022
      Company  Edinburgh Centre for Robotics
      Class Definition; Inter Process Comunication Class
"""

# Standard imports
import multiprocessing

class IPC_kuka_sharedVariables():    
    #ROBOT OUPUTS MOTION COMMANDS
    sharedTaskVel = multiprocessing.Array('d', (0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
    sharedSecondaryVel = multiprocessing.Array('d', (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
    sharedWeightsVehicle = multiprocessing.Array('d', (1000000000000.0, 1000000000000.0, 1000000000000.0))
    