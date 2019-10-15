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
    motor_config = rospy.get_param("motor_control/motor_config")

    if motor_config == "2WD":
        motors_pub = rospy.Publisher('roboclaw/speed_command', SpeedCommand, queue_size = 1)
    elif motor_config == "4WD":
        motor_front_pub = rospy.Publisher('roboclaw/speed_command/front', SpeedCommand, queue_size = 1)
        motor_back_pub = rospy.Publisher('roboclaw/speed_command/back', SpeedCommand, queue_size = 1)
    
    
    
    cmd_topic = rospy.get_param("motor_control/robot_cmd_topic")
    base_width = rospy.get_param("motor_control/base_width")
    ticks_per_meter = rospy.get_param("motor_control/ticks_per_meter")
    max_secs = rospy.get_param("motor_control/max_secs")

    #function triggered when a motor command is published
    def motor_cmd_callback(twist_msg):
        rospy.logdebug("Received motor command twist message")
        rospy.logdebug("Motor config: " + motor_config)
        #different implementation for different motor configs 
        if motor_config == "2WD":
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
        elif motor_config == "4WD":
            pass
            # motor_front_pub.publish()
            # motor_back_pub.publish()
        



    #at a high level the program just waits for messages on these topic and triggers the callback function when they arrive
    rospy.Subscriber(cmd_topic, Twist, motor_cmd_callback, queue_size = 1, buff_size = 16777216)


    #to keep the process from finishing while waiting for messages
    while not rospy.is_shutdown():
        rospy.spin()



