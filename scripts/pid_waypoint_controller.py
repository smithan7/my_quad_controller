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


class State(object):

  def __init__(self, xi, yi, zi, ti, fsi, psi, zsi, tsi):
    self.x = xi # locs
    self.y = yi # locs
    self.z = zi # locs
    self.t = ti # theta
    self.fs = fsi # forward speed
    self.ps = psi # perpendicular speed
    self.zs = zsi # alt vel
    self.ts = tsi # theta speed

  def print_state(self):
    print "x,y,z,t: ", self.x, ", ", self.y, ", ", self.z, ", ", self.t
    print "fs,ps,zs,ts: ", ", ", self.fs, ", ", self.ps, ", ", self.zs, ", ", self.ts


class PID_Controller(object):
  index = 0
  agent_type = 0
  cLoc = State(0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0) # x,y,z,t,fs,ps,zs,ts
  gLoc = State(0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0) # x,y,z,t,fs,ps,zs,ts
  err = State(0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0) # x,y,z,t,fs,ps,zs,ts
  pid_gains = State(1.0,1.0,0.75,2.0,4.0,4.0,0.1,0.75)
  max_state = State(50.0,50.0,20.0,50.0,4.0,1.0,0.5,1.0)
  min_state = State(-50.0,-50.0,2.0,-50.0,-1.0,-1.0,-0.5,-1.0)
  goal_index = -1
  
  update_rate = 100 # hz
  in_loop = False
  initialized = False
  goal_initialized = True
  path = []

  goal_tolerance = .5

  def init(self, ai, aa):
    # Initial values
    self.goal_altitude = aa
    self.gLoc.z = aa
    self.index = ai
    # Setup publisher
    self.pub_twist = rospy.Publisher('/uav' + str(self.index) + '/cmd_vel', Twist, queue_size=10)
    self.sub_odom = rospy.Subscriber('/uav' + str(self.index) + '/ground_truth/state', Odometry, self.odom_callback )
    self.goal_sub = rospy.Subscriber('/dmcts_' + str(self.index) + '/travel_goal', DMCTS_Travel_Goal, self.goal_callback )
    self.a_star_client = rospy.ServiceProxy('/dmcts_' + str(self.index) + '/costmap_bridge/a_star_path', Get_A_Star_Path)
    self.kinematic_client = rospy.ServiceProxy('/dmcts_' + str(self.index) + '/costmap_bridge/kinematic_path', Get_Kinematic_A_Star_Path)
    
    # which planner do I use?
    self.use_naive_pid = False
    self.use_a_star_path = True
    self.use_kinematic_path = False

    # map number, ensure I am planning from the right space
    self.map_num = -1

    # only needs to work once
    try:
      enable_motors = rospy.ServiceProxy('/uav' + str(self.index) + '/enable_motors', EnableMotors)
      enable_motors(True)
      self.motors_are_on = True

    except:
      rospy.logwarn('/uav' + str(self.index) + ' could not enable_motors')
      self.motors_are_on = False

    self.control_timer = rospy.Timer(rospy.Duration(0.01), self.control_callback)
    self.path_timer = rospy.Timer(rospy.Duration(2000.0), self.path_callback)


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
      #rospy.loginfo("control_callback: going into control")
      self.control()      


  def path_callback(self, event):
    rospy.loginfo("path_callback:: in")
    if self.use_naive_pid:
      return
    elif self.use_a_star_path:
      self.call_a_star_path_service(self.gLoc.x, self.gLoc.y)
    elif self.use_kinematic_path:
      self.call_kinematic_path_service(-15.0, -15.0)

  def call_a_star_path_service(self, gx, gy):
    rospy.logwarn("call_a_star_path_service:: in: gLoc: %i, %i", gx, gy)
    rospy.logwarn("call_a_star_path_service:: in: start: %i, %i", self.cLoc.x, self.cLoc.y)
    resp = self.a_star_client(round(self.cLoc.x), round(self.cLoc.y), gx, gy, self.map_num)
    rospy.logwarn("call_a_star_path_service:: out with path len %i", len(resp.xs))
    if resp.success:
      self.path = []
      for i in range(0, len(resp.xs)):
        # x,y,z,t,fs,ps,zs,ts
        if i+1 < len(resp.xs):
          a = State(resp.xs[i], resp.ys[i], self.goal_altitude, 0.0, 2.0, 0.0, 0.0, 0.0);
          self.path.append(a)
        else:
          a = State(resp.xs[i], resp.ys[i], self.goal_altitude, 0.0, 1.0, 0.0, 0.0, 0.0);
          self.path.append(a)
        #print "   ", resp.xs[i], ", ", resp.ys[i]
        
      return True
    else:
      rospy.logwarn("call_a_star_path_service:: fail")
      rospy.logwarn("   cLoc: %.2f, %.2f", self.cLoc.x, self.cLoc.y)
      rospy.logwarn("   gLoc: %.2f, %.2f", gx, gy)
      self.path = []
      return False

  def call_kinematic_path_service(self, gx, gy):
    
    resp = self.kinematic_client(self.cLoc.x, self.cLoc.y, self.cLoc.t, self.cLoc.fs, gx, gy, 0.0, 0.0, self.map_num)
    if resp.success:
      self.path = []
      for i in range(0, len(resp.xs)):
        # x,y,z,t,fs,ps,zs,ts
        a = State(resp.xs[i], resp.ys[i], self.goal_altitude, resp.thetas[i], resp.speeds[i], 0.0, 0.0, 0.0);
        self.path.append(a)
      return True
    else:
      self.path.clear()
      return False

  def goal_callback(self, msg):
    rospy.loginfo("goal_callback: goal_in: %0.2f, %0.2f", msg.x,msg.y)
    if self.use_naive_pid:
      self.path = []
      self.gLoc.x = msg.x
      self.gLoc.y = msg.y
      self.goal_initialized = True
    elif self.use_a_star_path:
      if self.call_a_star_path_service(msg.x, msg.y):
        self.goal_initialized = True
        a = State(msg.x, msg.y, self.goal_altitude, 0.0, 0.0, 0.0, 0.0, 0.0);
        self.path.append(a)
      else:
        self.goal_initialized = False
    elif self.use_kinematic_path:
      if self.call_kinematic_path_service(msg.x, msg.y):
        self.goal_initialized = True
        a = State(msg.x, msg.y, self.goal_altitude, 0.0, 0.0, 0.0, 0.0, 0.0);
        self.path.append(a)
      else:
        self.goal_initialized = False
    else:
      rospy.logerr("PID_Controller::goal_callback: NO PLANNING METHOD PROVIDED")
      self.goal_initialized = False
    

  def odom_callback(self, msg):
    if self.in_loop == False:
      yaw = self.quaternions_to_RPY([msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])
      self.cLoc.x = msg.pose.pose.position.x
      self.cLoc.y = msg.pose.pose.position.y
      self.cLoc.z = msg.pose.pose.position.z
      self.cLoc.t = yaw 
      self.cLoc.fs = math.sqrt( msg.twist.twist.linear.x**2 + msg.twist.twist.linear.y**2)
      self.cLoc.zs = msg.twist.twist.linear.z
      self.cLoc.ts = msg.twist.twist.angular.z
    
    if self.initialized == False:
      self.gLoc.x = self.cLoc.x
      self.gLoc.y = self.cLoc.y
      self.gLoc.z = 5.0
      self.initialized = True

  def limit_goals(self):
    self.gLoc.x = min(max(self.gLoc.x, self.min_state.x), self.max_state.x)
    self.gLoc.y = min(max(self.gLoc.y, self.min_state.y), self.max_state.y)

  def limit_twist_out(self, pid):
    pid.fs = min(max(pid.fs, self.min_state.fs), self.max_state.fs)
    pid.ps = min(max(pid.ps, self.min_state.ps), self.max_state.ps)
    pid.zs = min(max(pid.zs, self.min_state.zs), self.max_state.zs)
    pid.ts = min(max(pid.ts, self.min_state.ts), self.max_state.ts)
    return pid

  def update_goal(self):
    # check if I have a long path
    if len(self.path) > 1 and not self.use_naive_pid:
      # check if I am at my goal
      d1 = math.sqrt((self.cLoc.x-self.path[0].x)**2 + (self.cLoc.y-self.path[0].y)**2)
      if d1 < self.goal_tolerance:
        self.gLoc.x = self.path[1].x
        self.gLoc.y = self.path[1].y
        self.gLoc.z = self.path[1].z
        self.gLoc.t = self.path[1].t
        self.gLoc.fs = self.path[1].fs
      else:
        # check if I am some how closer to the next wp
        d2 = math.sqrt((self.cLoc.x-self.path[1].x)**2 + (self.cLoc.y-self.path[1].y)**2)
        if d2 < d1:
          self.gLoc.x = self.path[1].x
          self.gLoc.y = self.path[1].y
          self.gLoc.z = self.path[1].z
          self.gLoc.t = self.path[1].t
          self.gLoc.fs = self.path[1].fs

  def control(self):
    if self.initialized == False:
      return

    if self.goal_initialized == False:
      self.gLoc.x = self.cLoc.x
      self.gLoc.y = self.cLoc.y
      self.gLoc.z = self.cLoc.z
    elif len(self.path) > 1:
      self.update_goal()

    self.in_loop = True
    
    self.limit_goals()


    #self.cLoc.x = 33
    #self.cLoc.y = 81
    #self.cLoc.t = -1.88

    #print "******************** Goal *******************************"
    #self.gLoc.print_state()
    #print "******************** cLoc *******************************"
    #self.cLoc.print_state()

    # theta = 0 -> 2*pi
    self.cLoc.t += 6.28318530718
    self.cLoc.t = self.cLoc.t % 6.28318530718

    # move goal to local frame
    [g_dx, g_dy, self.gLoc.t] = self.position_from_a_to_b(self.cLoc, self.gLoc)
    [l_dx, l_dy] = self.global_to_local_frame(g_dx, g_dy, self.cLoc.t)
    self.global_to_local_vel()
    

    # theta 0 -> 2 pi
    self.gLoc.t += 6.28318530718
    self.gLoc.t = self.gLoc.t % 6.28318530718

    # fix roll over
    if self.cLoc.t < 3.14159265359/2.0 and self.gLoc.t > 3.0*3.14159265359/4.0:
      self.gLoc.t = self.gLoc.t - 6.28318530718
    elif self.cLoc.t > 3.0*3.14159265359/4.0 and self.gLoc.t < 3.14159265359/2.0:
      self.gLoc.t = self.gLoc.t + 6.28318530718
    
    # get error
    self.err.x = l_dx
    self.err.y = l_dy
    self.err.z = self.gLoc.z - self.cLoc.z
    self.err.t = self.gLoc.t - self.cLoc.t

    if abs(self.err.x) + abs(self.err.y) < 1.0:
      self.err.t = 0.0

    if abs(self.err.z) > 1.0:
      self.err.x = 0.0
      self.err.y = 0.0
    
    self.err.fs = self.gLoc.fs - self.cLoc.fs
    self.err.zs = self.gLoc.zs - self.cLoc.zs
    self.err.ts = self.gLoc.ts - self.cLoc.ts
    
    if abs(self.err.fs) < self.max_state.fs:
      self.err.fs = 0
    

    pid = State(0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0)
    pid.fs = self.pid_gains.fs * self.err.fs + self.pid_gains.x * self.err.x
    pid.ps = self.pid_gains.ps * self.err.ps + self.pid_gains.y * self.err.y
    pid.zs = self.pid_gains.zs * self.err.zs + self.pid_gains.z * self.err.z
    pid.ts = self.pid_gains.ts * self.err.ts + self.pid_gains.t * self.err.t
    
    #print "***************************** pid ***************************"
    #pid.print_state()

    pid = self.limit_twist_out(pid)
    
    twist = Twist()  
    twist.linear.x = pid.fs
    twist.linear.y = pid.ps
    twist.linear.z = pid.zs
    twist.angular.z = pid.ts
    #print "twist: ", twist
    self.pub_twist.publish(twist) 
    self.in_loop = False

  def global_to_local_frame(self, gx, gy, gw):
    lx = gx*math.cos(gw) + gy*math.sin(gw)
    ly = -gx*math.sin(gw) + gy*math.cos(gw)

    return [lx, ly]

  def global_to_local_vel(self):
    vx = self.cLoc.fs*math.cos(self.cLoc.t) + self.cLoc.ps*math.sin(self.cLoc.t)
    vy = -self.cLoc.fs*math.sin(self.cLoc.t) + self.cLoc.ps*math.cos(self.cLoc.t)
    self.cLoc.fs = vx
    self.cLoc.ps = vy

  def position_from_a_to_b( self, a, b ):
    x = b.x - a.x
    y = b.y - a.y
    heading = math.atan2(y, x)

    return [x,y,heading]

  def at_goal(self):
    self.err = math.sqrt((self.gLoc.x - self.cLoc.x)**2 + (self.gLoc.y - self.cLoc.y)**2  + (self.gLoc.z - self.cLoc.z)**2)
    error_yaw = abs(self.gLoc.t - self.cLoc.t)
    if (self.err < 1.0 and error_yaw < 8.0):
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
