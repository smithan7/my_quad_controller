#!/usr/bin/env python


import rospy, math
import numpy as np
from random import *
import sys, termios, tty, select, os

from custom_messages.msg import DMCTS_Travel_Goal
from custom_messages.srv import Get_A_Star_Path, Get_Kinematic_A_Star_Path
from hector_uav_msgs.srv import EnableMotors
from hector_uav_msgs.msg import PoseAction
from geometry_msgs.msg import Twist, Pose, Point, PoseStamped
from nav_msgs.msg import Odometry, Path

class PID_Controller(object):
  index = 0
  agent_type = 0
  p_z = 0.75
  d_z = 0.1
  max_z = 50.0
  min_z = 2.0
  max_dz = 1.0
  min_dz = -1.0
  initialized = False
  g_z = 2.0
  g_dz = 0.0
  c_z = 0.0
  c_dz = 0.0
  handover_control = False
  
  def init(self, ai, aa):
    # Initial values
    self.g_z = aa
    self.index = ai
    # Setup publisher
    self.pub_twist = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    self.sub_twist = rospy.Subscriber('/cmd_vel_from_move_base', Twist, self.twist_callback)
    self.sub_odom = rospy.Subscriber('/ground_truth/state', Odometry, self.odom_callback )
    self.control_timer = rospy.Timer(rospy.Duration(0.01), self.control_callback)

    # only needs to work once
    try:
      enable_motors = rospy.ServiceProxy('/enable_motors', EnableMotors)
      enable_motors(True)
      self.motors_are_on = True

    except:
      rospy.logwarn(' could not enable_motors')
      self.motors_are_on = False

  def twist_callback(self, msg):
    self.handover_control = True
    if not self.motors_are_on:
      try:
        enable_motors = rospy.ServiceProxy('/enable_motors', EnableMotors)
        enable_motors(True)
        self.motors_are_on = True
      except:
        rospy.logwarn(' could not enable_motors')
        rospy.sleep(3)
        self.motors_are_on = False
    else:
      self.control(msg)

  def odom_callback(self, msg):
    self.c_z = msg.pose.pose.position.z
    self.c_dz = msg.twist.twist.linear.z
    self.initialized = True
    
  def control_callback(self, event):
    if self.initialized == False or self.handover_control == True:
      #print "handover_control: ", self.handover_control
      #print "initialized: ", self.initialized
      return
    
    # only needs to work once
    if not self.motors_are_on:
        try:
          enable_motors = rospy.ServiceProxy('/enable_motors', EnableMotors)
          enable_motors(True)
          self.motors_are_on = True

        except:
          rospy.logwarn(' could not enable_motors')
          self.motors_are_on = False

    # get error
    err_z = self.g_z - self.c_z
    err_dz = self.g_dz - self.c_dz
    
    
    pid_z= self.d_z * err_dz + self.p_z * err_z
    pid_z = min(max(pid_z, self.min_dz), self.max_dz)
    
    # don't travel while at the wrong alt or pointing in the wrong direction
    twist = Twist()  
    twist.linear.x = 0.0
    twist.linear.y = 0.0
    twist.linear.z = pid_z
    twist.angular.z = 0.0
    self.pub_twist.publish(twist)
    

  def control(self, msg):
    if self.initialized == False:
      return

    # get error
    err_z = self.g_z - self.c_z
    err_dz = self.g_dz - self.c_dz
    
    
    pid_z= self.d_z * err_dz + self.p_z * err_z
    pid_z = min(max(pid_z, self.min_dz), self.max_dz)
    
    # don't travel while at the wrong alt or pointing in the wrong direction
    if abs(err_z) > 1.0:
        twist = Twist()  
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = pid_z
        twist.angular.z = 0.0
        #print "twist_stationary: ", twist
        self.pub_twist.publish(twist)
    else:
        msg.linear.z = pid_z
        self.pub_twist.publish(msg)
        #print "twist_msg: ", msg
        self.pub_twist.publish(msg)
    
if __name__ == '__main__':
  rospy.init_node('pid_postion_control')
  pid = PID_Controller()
  
  pid.init(0, 3.0)
  rospy.loginfo("Quad PID Controller::initialized")
  rospy.spin()
