#!/usr/bin/env python3
"""
      @author   Carlos Suarez Zapico carlossuarezzapico@gmail.com
 
      @internal
      Created  22-Dec-2022
      Company  Edinburgh Centre for Robotics
      Class Definition; Processes Teleoperation
"""

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import rospy
import numpy as np
from random import random
import time
from pynput.keyboard import Key
from pynput.keyboard import Listener
from processes.IPC_processes_kuka import IPC_kuka_sharedVariables


class teleop_process(IPC_kuka_sharedVariables):
 
    def ps4_secondary_vel_thread_acc_limit_smooth(self): 
        rospy.init_node("ps4_secondary_acc_limit_smooth", anonymous=True)
        factor_linear =1.5
        vel_pre_x =0.0; vel_pre_y =0.0; vel_pre_yaw =0.0
        secondary_saturated_x =0.0; secondary_saturated_y =0.0; secondary_saturated_yaw =0.0
        accX = 0.2; accY=0.2; accYaw =1.0
        time_last_action = time.time()
        sampling_control = 0.01

        while (not (rospy.is_shutdown()) or self.sharedFINISH.value):  
            if ((time.time()- time_last_action)>sampling_control):
                vel_max = self.sharedPS4MaxVelbase.value

                secondary_saturated_x = self.aux_tools.VAL_SAT( factor_linear*self.sharedPS4TWIST[0], vel_max, -vel_max)
                secondary_saturated_y = self.aux_tools.VAL_SAT( factor_linear*self.sharedPS4TWIST[1], vel_max, -vel_max)
                secondary_saturated_yaw = self.aux_tools.VAL_SAT( self.sharedPS4TWIST[2], 0.9, -0.9)

                secondary_saturated_x =self.aux_tools.VAL_SAT( secondary_saturated_x, vel_pre_x + accX*sampling_control,  vel_pre_x - accX*sampling_control)
                secondary_saturated_y =self.aux_tools.VAL_SAT( secondary_saturated_y, vel_pre_y + accY*sampling_control,  vel_pre_y - accY*sampling_control)
                secondary_saturated_yaw =self.aux_tools.VAL_SAT( secondary_saturated_yaw, vel_pre_yaw + accYaw*sampling_control,  vel_pre_yaw - accYaw*sampling_control)
                
                vel_pre_x =secondary_saturated_x; vel_pre_y =secondary_saturated_y; vel_pre_yaw =secondary_saturated_yaw;
                secondary_vel = np.array([secondary_saturated_x, secondary_saturated_y, secondary_saturated_yaw, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
                time_last_action = time.time()
                self.aux_tools.copy_npArray_to_sharedArray(secondary_vel, self.sharedSecondaryVel)

    def pub_teleop_primary_secondary(self):
        rospy.init_node("publish_primary_secondary", anonymous=True) 
        pub = rospy.Publisher('primary_secondary_velocity', Float64MultiArray, queue_size=10)
        msg = Float64MultiArray()
        time_last_action = time.time()
        sampling_control = 0.05
        while (not rospy.is_shutdown()): 
                if ((time.time()- time_last_action)>sampling_control):
                        time_last_action = time.time()
                        msg.data = [self.sharedTaskVel[0], self.sharedTaskVel[1], self.sharedTaskVel[2], self.sharedTaskVel[3], self.sharedTaskVel[4], self.sharedTaskVel[5],\
                                    self.sharedSecondaryVel[0], self.sharedSecondaryVel[1], self.sharedSecondaryVel[2], self.sharedSecondaryVel[3], self.sharedSecondaryVel[4], 
                                    self.sharedSecondaryVel[5], self.sharedSecondaryVel[6], self.sharedSecondaryVel[7], self.sharedSecondaryVel[8], self.sharedSecondaryVel[9]] 
                        pub.publish(msg)



    def teleop_thread(self): 
        rospy.init_node("teleoperated_xy", anonymous=True)   
        # Subscribers
        with Listener(on_press=self.Task_keyboard_press, on_release=self.Task_keyboard_release) as listener: listener.join() 

        while (not rospy.is_shutdown()):             
            #with Listener(on_press=self.Task_Task_keyboard_press, on_release=self.Task_keyboard_release) as listener: listener.join() 
            print("waiting for motion commmands")
            
    def Task_keyboard_release(self, key):    
        if key == Key.esc:
            return False
        try:
            key_released = key.char
        except:
            return
        if key.char == 'a':        
                self.sharedTaskVel[0] = 0.0

                print('a released')
        elif key.char == 'd':        
                self.sharedTaskVel[0] = 0.0
       
                print('d released')
        elif key.char == 'w':        
 
                self.sharedTaskVel[1] = 0.0
       
                print('w released')
        elif key.char == 's':        

                self.sharedTaskVel[1] = 0.0
      
                print('s released')            
        elif key.char == 'q':        

                self.sharedTaskVel[2] = 0.0
    
                print('j released')
        elif key.char == 'z':        

                self.sharedTaskVel[2] = 0.0

                print('l released')
        
                
                    
    def Task_keyboard_press(self, key):   
        try:
            key_pressed = key.char
        except:
            return
        if key.char == 'a':        
                self.sharedTaskVel[0] = 0.1

                print('a pressed')
        elif key.char == 'd':        
                self.sharedTaskVel[0] = -0.1
   
                print('d pressed')
        elif key.char == 'w':        

                self.sharedTaskVel[1] = 0.1
              
                print('w pressed')
        elif key.char == 's':        

                self.sharedTaskVel[1] = -0.1
              
                print('s pressed')
        elif key.char == 'q':        

                self.sharedTaskVel[2] = 0.1
    
                print('j pressed')
        elif key.char == 'z':        

                self.sharedTaskVel[2] = -0.1

                print('l pressed')
        
    
    def teleop_secondary_thread(self): 
        rospy.init_node("teleoperated_xy", anonymous=True)   
        # Subscribers
        with Listener(on_press=self.Secondary_keyboard_press, on_release=self.Secondary_keyboard_release) as listener: listener.join() 
        while (not rospy.is_shutdown()):             
            #with Listener(on_press=self.Task_Task_keyboard_press, on_release=self.Task_keyboard_release) as listener: listener.join() 
            print("waiting for motion commmands")
            
    def Secondary_keyboard_release(self, key):    
        if key == Key.esc:
            return False
        try:
            key_released = key.char
        except:
            return
        if key.char == 'i':        
                self.sharedSecondaryVel[0] = 0.0

                print('a released')
        elif key.char == 'k':        
                self.sharedSecondaryVel[0] = 0.0
  
                print('d released')
        elif key.char == 'j':        

                self.sharedSecondaryVel[1] = 0.0

                print('w released')
        elif key.char == 'l':        
       
                self.sharedSecondaryVel[1] = 0.0
   
                print('s released')            
        elif key.char == 'u':        

                self.sharedSecondaryVel[2] = 0.0
                print('j released')
        elif key.char == 'm':        
     

                self.sharedSecondaryVel[2] = 0.0
                print('l released')

                
                    
    def Secondary_keyboard_press(self, key):   
        try:
            key_pressed = key.char
        except:
            return
        if key.char == 'i':        
                self.sharedSecondaryVel[0] = 0.2
                print('a pressed')
        elif key.char == 'k':        
                self.sharedSecondaryVel[0] = -0.2
                print('d pressed')
        elif key.char == 'j':        
                self.sharedSecondaryVel[1] = 0.2

                print('w pressed')
        elif key.char == 'l':        
 
                self.sharedSecondaryVel[1] = -0.2
              
                print('s pressed')
        elif key.char == 'u':        

                self.sharedSecondaryVel[2] = 0.2
                print('j pressed')
        elif key.char == 'm':        

                self.sharedSecondaryVel[2] = -0.2
                print('l pressed')
    