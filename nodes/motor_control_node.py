#!/usr/bin/env python2

# import ros libraries
import rospy
import roslib

from geometry_msgs.msg import Twist
from roboclaw_driver.msg import SpeedCommand

# import miscellaneous modules
import os
import time
from math import *

if __name__ == '__main__':

    rospy.init_node('motor_control', anonymous=True)
    FourWD = rospy.get_param("motor_control/4WD")
    print(FourWD)
    FourWD = bool(FourWD)
    print(FourWD)

    if not FourWD:
        motors_pub = rospy.Publisher('roboclaw/speed_command', SpeedCommand, queue_size = 1)
    elif FourWD:
        motor_right_pub = rospy.Publisher('roboclaw/speed_command/right', SpeedCommand, queue_size = 1)
        motor_left_pub = rospy.Publisher('roboclaw/speed_command/left', SpeedCommand, queue_size = 1)
    
    
    
    cmd_topic = rospy.get_param("motor_control/robot_cmd_topic")
    base_width = rospy.get_param("motor_control/base_width")
    ticks_per_meter = rospy.get_param("motor_control/ticks_per_meter")
    max_secs = rospy.get_param("motor_control/max_secs")

    #function triggered when a motor command is published
    def motor_cmd_callback(twist_msg):
        rospy.logdebug("Received motor command twist message")
        rospy.logdebug("FourWD: ", FourWD)
        #different implementation for different motor configs 
        if not FourWD:
            #take the twist message
            linear_x = twist_msg.linear.x #check the driver node does speed limiting
            #angular components are in radians/second
            #to convert to m/s we multiply by m/rad ie 

            #if rotating at w then the rim is traveling at w*r
            #so m/rad is equal to the radius 
            vr = linear_x + twist_msg.angular.z * base_width / 2.0  # m/s
            vl = linear_x - twist_msg.angular.z * base_width / 2.0

            vr_ticks = int(vr * ticks_per_meter)  # ticks/s or qpps
            vl_ticks = int(vl * ticks_per_meter)
            msg = SpeedCommand()
            msg.m1_qpps = vr_ticks
            msg.m2_qpps = vl_ticks
            msg.max_secs = max_secs
            motors_pub.publish(msg)

        #fill this out if needed...
        elif FourWD:
            #take the twist message
            linear_x = twist_msg.linear.x #check the driver node does speed limiting
            #angular components are in radians/second
            #to convert to m/s we multiply by m/rad ie 

            #if rotating at w then the rim is traveling at w*r
            #so m/rad is equal to the radius 
            vr = linear_x + twist_msg.angular.z * base_width / 2.0  # m/s
            vl = linear_x - twist_msg.angular.z * base_width / 2.0

            vr_ticks = int(vr * ticks_per_meter)  # ticks/s or qpps
            vl_ticks = int(vl * ticks_per_meter)
            msgR = SpeedCommand()
            msgL = SpeedCommand()
            msgR.m1_qpps = vr_ticks
            msgR.m2_qpps = vr_ticks
            msgR.max_secs = max_secs
            msgL.m1_qpps = vl_ticks
            msgL.m2_qpps = vl_ticks
            msgL.max_secs = max_secs
            motor_right_pub.publish(msgR)
            motor_left_pub.publish(msgL)
        



    #at a high level the program just waits for messages on these topic and triggers the callback function when they arrive
    rospy.Subscriber(cmd_topic, Twist, motor_cmd_callback, queue_size = 1, buff_size = 16777216)


    #to keep the process from finishing while waiting for messages
    while not rospy.is_shutdown():
        rospy.spin()



