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
  loc = np.array([0.0,0.0,0.0,0.0]) # x,y,z,w
  vel = np.array([0.0,0.0,0.0,0.0]) # x,y,z,w
  goal_index = -1
  goal_loc = np.array([0.0,0.0,5.0, 0.0]) # x,y,z,w
  goal_vel = np.array([0.0,0.0,0.0,0.0]) # x,y,z,w

  proportional = np.array([1.0, 1.0, 0.75, 2.0])
  derivative = np.array([4.0, 0.0, 0.1, 0.75])
  
  max_vel = [4.0, 0.5, 1.0]
  min_vel = [-1.0, -0.5, -1.0]

  max_loc = [50, 50, 20]
  min_loc = [-50, -50, 2]

  update_rate = 100 # hz
  in_loop = False
  initialized = False
  goal_initialized = True

  def init(self, ai, aa):
    # Initial values
    self.goal_altitude = aa
    self.goal_loc[2] = aa
    self.index = ai
    # Setup publisher
    self.pub_twist = rospy.Publisher('/uav' + str(self.index) + '/cmd_vel', Twist, queue_size=10)
    self.sub_odom = rospy.Subscriber('/uav' + str(self.index) + '/ground_truth/state', Odometry, self.odom_callback )
    self.goal_sub = rospy.Subscriber('/dmcts_' + str(self.index) + '/travel_goal', DMCTS_Travel_Goal, self.goal_callback )
    self.a_star_client = rospy.Subscriber('/dmcts_' + str(self.index) + '/costmap_bridge/a_star_path', Get_A_Star_Path)
    self.kinematic_client = rospy.ServiceProxy('/dmcts_' + str(self.index) + '/costmap_bridge/kinematic_path', Get_Kinematic_A_Star_Path)
    
    # only needs to work once
    try:
      enable_motors = rospy.ServiceProxy('/uav' + str(self.index) + '/enable_motors', EnableMotors)
      enable_motors(True)
      self.motors_are_on = True

    except:
      rospy.logwarn('/uav' + str(self.index) + ' could not enable_motors')
      self.motors_are_on = False

    self.control_timer = rospy.Timer(rospy.Duration(0.01), self.control_callback)


  def control_callback(self, event):
    if not self.motors_are_on:
      try:
        enable_motors = rospy.ServiceProxy('/uav' + str(self.index) + '/enable_motors', EnableMotors)
        enable_motors(True)
        self.motors_are_on = True

      except:
        rospy.logwarn('/uav' + str(self.index) + ' could not enable_motors')
        rospy.sleep(3)
        self.motors_are_on = False
    else:
      self.control()      

  def a_star_path_callback(self, msg):
    self.path_xs = msg.xs
    self.path_ys = msg.ys

    self.goal_loc = np.array([path_xs[0], path_ys[0], self.goal_altitude, 0.0])
    self.goal_vel = np.array([0.0, 0.0, 0.0, 0.0])
    self.goal_initialized = True

  def kinematic_path_callback(self, msg):
    self.path_xs = msg.xs
    self.path_ys = msg.ys
    self.thetas = msg.thetas
    self.speeds = msg.speeds

    self.goal_loc = np.array([path_xs[0], path_ys[0], self.goal_altitude, self.thetas[0]])
    self.goal_vel = np.array([self.speeds[0]*cos(self.thetas[0]), self.speeds[0]*sin(self.thetas[0]), 0.0, 0.0])
    self.goal_initialized = True

  def goal_callback(self, msg):
    self.goal_loc = np.array([msg.x, msg.y, self.goal_altitude, 0.0])
    self.goal_vel = np.array([0.0, 0.0, 0.0, 0.0])
    #rospy.logerr("got goal loc: %.2f, %.2f", msg.x, msg.y)
    #yaw = self.quaternions_to_RPY([p.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])
    #self.goal_loc = np.array([p.pose.pose.position.x, p.pose.pose.position.y, p.pose.pose.position.z, yaw]) 
    #self.goal_vel = np.array([p.twist.twist.linear.x, p.twist.twist.linear.y, p.twist.twist.linear.z, msg.twist.twist.angular.z])
    self.goal_initialized = True
    

  def odom_callback(self, msg):
    if self.in_loop == False:
      yaw = self.quaternions_to_RPY([msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])
      self.loc = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z, yaw]) 
      self.vel = np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z, msg.twist.twist.angular.z])
    
    if self.initialized == False:
      self.goal_loc[0] = self.loc[0]
      self.goal_loc[1] = self.loc[1]
      self.goal_loc[2] = 5.0
      self.initialized = True


  def limit_goals(self):
    for i in range(0,2):
      self.goal_loc[i] = min(max(self.goal_loc[i], self.min_loc[i]), self.max_loc[i])

    for i in range(0,2):
      self.goal_vel[i] = min(max(self.goal_vel[i], self.min_vel[i]), self.max_vel[i])

    self.goal_vel[3] = 0.0

  def limit_twist_out(self, pid):
    for i in range(0,3):
      pid[i] = min(max(pid[i], self.min_vel[i]), self.max_vel[i])

    pid[0] = pid[0]*(1-abs(abs(self.goal_loc[3] - self.loc[3]) / 6.28))
    pid[1] = pid[1]*(1-abs(abs(self.goal_loc[3] - self.loc[3]) / 6.28))
    pid[0] = pid[0]*(1-abs(abs(self.goal_loc[2] - self.loc[2])))
    pid[1] = pid[1]*(1-abs(abs(self.goal_loc[2] - self.loc[2])))
    
    return pid

  def control(self):
    if self.initialized == False:
      return

    if self.goal_initialized == False:
      self.goal_loc[0] = self.loc[0]
      self.goal_loc[1] = self.loc[1]
      self.goal_loc[2] = self.altitude
    elif len(self.path) > 1:
        this needs to check if path length is long and iterate along path_xs
        check if at point or closer to next point in path
      
    self.in_loop = True
    #print "loc: ", self.loc
    #print "vel: ", self.vel
    #print "goal: ", self.goal_loc
    #print "goal_vel: ", self.goal_vel

    self.limit_goals()


    #self.loc[0] = 33
    #self.loc[1] = 81
    #self.loc[3] = -1.88

    #print "Goal: ", round(self.goal_loc[0],2), ", ", round(self.goal_loc[1],2)
    #print "Loc: ", round(self.loc[0],2), ", ", round(self.loc[1],2), ", ", round(self.loc[3]*67.3,2)

    # theta = 0 -> 2*pi
    self.loc[3] += 6.28318530718
    self.loc[3] = self.loc[3] % 6.28318530718

    # move goal to local frame
    [g_dx, g_dy, self.goal_loc[3]] = self.position_from_a_to_b(self.loc, self.goal_loc)
    [l_dx, l_dy] = self.global_to_local_frame(g_dx, g_dy, self.loc[3])

    #print "gd: ", round(g_dx,2), ", ", round(g_dy,2), ", ", round(self.goal_loc[3]*57.3,2)
    #print "ld: ", round(l_dx,2), ", ", round(l_dy,2)


    # theta 0 -> 2 pi
    self.goal_loc[3] += 6.28318530718
    self.goal_loc[3] = self.goal_loc[3] % 6.28318530718

    # fix roll over
    if self.loc[3] < 3.14159265359/2.0 and self.goal_loc[3] > 3.0*3.14159265359/4.0:
      self.goal_loc[3] = self.goal_loc[3] - 6.28318530718
    elif self.loc[3] > 3.0*3.14159265359/4.0 and self.goal_loc[3] < 3.14159265359/2.0:
      self.goal_loc[3] = self.goal_loc[3] + 6.28318530718
    #print "loc[3]: ", self.loc[3]

    error_loc = self.goal_loc - self.loc
    error_loc[0] = l_dx
    error_loc[1] = l_dy


    if abs(error_loc[0]) + abs(error_loc[1]) < 1.0:
      error_loc[3] = 0.0

    if abs(error_loc[2]) > 1.0:
      error_loc[0] = 0.0
      error_loc[1] = 0.0
    
    #print "error_loc: ", round(error_loc[0],2), ", ", round(error_loc[1],2), ", ", round(error_loc[2],2), ", ", round(error_loc[3]*57.3,2)
    p = np.multiply(self.proportional, error_loc)
    #print "proportional: ", round(p[0],2), ", ", round(p[1],2), ", ", round(p[2],2), ", ", round(p[3],2)

    self.global_to_local_vel()
    error_vel = self.goal_vel - self.vel
    if abs(error_vel[0]) < self.max_vel[0]:
      error_vel[0] = 0
    #print "error_vel: ", round(error_vel[0],2), ", ", round(error_vel[1],2), ", ", round(error_vel[2],2), ", ", round(error_vel[3],2)
    d = np.multiply(np.sign(error_vel), np.multiply(self.derivative, error_vel))
    #print "sign(error_vel): ", np.sign(error_vel)
    #print "self.vel: ", round(self.vel[0],2), ", ", round(self.vel[1],2), ", ", round(self.vel[2],2), ", ", round(self.vel[3],2)
    #print "derivative: ", round(d[0],2), ", ", round(d[1],2), ", ", round(d[2],2), ", ", round(d[3],2)

    pid = p + d 
    #print "PID: ", round(pid[0],2), ", ", round(pid[1],2), ", ", round(pid[2],2), ", ", round(pid[3],2)

    pid = self.limit_twist_out(pid)
    #print "PID: ", round(pid[0],2), ", ", round(pid[1],2), ", ", round(pid[2],2), ", ", round(pid[3],2)

    twist = Twist()  
    twist.linear.x = pid[0]
    twist.linear.y = pid[1]
    twist.linear.z = pid[2]
    twist.angular.z = pid[3]
    #print "twist: ", twist
    self.pub_twist.publish(twist) 

    #ch = raw_input()
    self.in_loop = False

  def global_to_local_frame(self, gx, gy, gw):
    lx = gx*math.cos(gw) + gy*math.sin(gw)
    ly = -gx*math.sin(gw) + gy*math.cos(gw)

    return [lx, ly]

  def global_to_local_vel(self):
    vx = self.vel[0]*math.cos(self.loc[3]) + self.vel[1]*math.sin(self.loc[3])
    vy = -self.vel[0]*math.sin(self.loc[3]) + self.vel[1]*math.cos(self.loc[3])
    self.vel[0] = vx
    self.vel[1] = vy

  def position_from_a_to_b( self, a, b ):
    x = b[0] - a[0]
    y = b[1] - a[1]
    heading = math.atan2(y, x)

    return [x,y,heading]

  def at_goal(self):
    error_loc = math.sqrt((self.goal_loc[0] - self.loc[0])**2 + (self.goal_loc[1] - self.loc[1])**2  + (self.goal_loc[2] - self.loc[2])**2)
    error_yaw = abs(self.goal_loc[3] - self.loc[3])
    if (error_loc < 1.0 and error_yaw < 8.0):
      return True
    else:
      return False

  def quaternions_to_RPY( self, q ):
    yaw  = math.atan2(2.0 * (q[3] * q[2] + q[0] * q[1]) , 1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2]) )
    #pitch = math.asin(2.0 * (q[2] * q[0] - q[3] * q[1]) )
    #roll   = math.atan2(2.0 * (q[3] * q[0] + q[1] * q[2]) , - 1.0 + 2.0 * (q[0] * q[0] + q[1] * q[1]) )
    return yaw
    
if __name__ == '__main__':
  rospy.init_node('pid_postion_control')
  pid = PID_Controller()
  ai = rospy.get_param('agent_index')
  #aa = rospy.get_param('agent_altitude')
  aa = float(ai+1)*2.5 + 5.0

  rospy.loginfo("Quad PID Controller::initializing")
  rospy.loginfo(" Quad PID Controller::agent index: %i", ai)
  rospy.loginfo(" Quad PID Controller::agent altitude: %i", aa)
  
  pid.init(ai, aa)
  rospy.loginfo("Quad PID Controller::initialized")
  rospy.spin()
