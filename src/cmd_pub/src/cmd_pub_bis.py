#!/usr/bin/env python

import logging
import sys
import time
import rospy
import math
import vesc_msgs
from math import sin, cos
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3, Twist
from nav_msgs.msg import Odometry
from vesc_msgs.msg import VescStateStamped
rospy.init_node('cmd_to_erpm', anonymous= True)
pub_left=rospy.Publisher('/left_wheel/commands/motor/speed',Float64, queue_size=5)
pub_right=rospy.Publisher('/right_wheel/commands/motor/speed',Float64, queue_size=5)
wr = 0.0
wl = 0.0

def cmd_vel_callback(speedl):
  global wr
  global wl
  v = speedl.linear.x
  omega = speedl.angular.z
  L = 0.525
  r = 0.1
  wr = (v+omega*L)/(2*3.14*r)*60*7*6   ### [(speed(tr/s)*60)(tr/min)]*7*6=RPM*7*Courroie=ERPM*courroie
  wl = (v-omega*L)/(2*3.14*r)*60*7*6   ### [(speed(tr/s)*60)(tr/min)]*7*6=RPM*7*courroie=ERPM*courroie
  if wr < 0 :
    wr = 0
  if wl < 0 :
    wl = 0

def cmd_to_erpm_callback():
  rospy.Subscriber('/cmd_vel_to_rpm', Twist, cmd_vel_callback)
  rate = rospy.Rate(10)
  while not rospy.is_shutdown():
    global wr
    global wl
    pub_right.publish(-wr)
    pub_left.publish(wl)
    wr = 0
    wl = 0
    rate.sleep()
		
		
		
		

if __name__=='__main__' :
	try:
		cmd_to_erpm_callback()
	except rospy.ROSInterruptException:
		pass


