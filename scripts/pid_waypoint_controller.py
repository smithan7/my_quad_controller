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
from visualization_msgs.msg import Marker

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
  carrot = State(0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0) # x,y,z,t,fs,ps,zs,ts
  goal = State(0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0) # x,y,z,t,fs,ps,zs,ts
  #err = State(0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0) # x,y,z,t,fs,ps,zs,ts
  pid_cruise_gains = State(1.0,0.1,0.75,2.0,8.0,4.0,0.1,0.75)
  pid_approach_gains = State(0.5,0.5,0.75,2.0,1.0,1.0,0.1,0.75)
  max_state = State(50.0,50.0,30.0,50.0,2.0,0.2,1.0,1.0)
  min_state = State(-50.0,-50.0,2.0,-50.0,-1.0,-0.2,-1.0,-1.0)
  pid_mean = State(0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0)
  goal_index = -1
  
  update_rate = 100 # hz
  in_loop = False
  initialized = False
  goal_initialized = True
  path = []

  goal_tolerance = 0.5
  control_radius = 5.0

  def init(self, ai, aa, cs):
    self.max_state.fs = cs
    self.min_state.fs = -cs;
    # Initial values
    self.goal.z = aa
    self.cruising_speed = cs
    self.index = ai
    # Setup publisher
    self.pub_twist = rospy.Publisher('/uav' + str(self.index) + '/cmd_vel', Twist, queue_size=10)
    self.pub_marker = rospy.Publisher('/PID_Control/path_markers', Marker, queue_size=1)
    # Setup subscriber
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

    self.control_timer = rospy.Timer(rospy.Duration(0.01), self.control_timer_callback)
    # dont need, goal callbac covers self.path_timer = rospy.Timer(rospy.Duration(20000.0), self.path_timer_callback)


  def control_timer_callback(self, event):
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
      d = math.sqrt((self.cLoc.x-self.goal.x)**2 + (self.cLoc.y-self.goal.y)**2)
      #print "D: ", d, " with goal: ", self.goal.x, ", ", self.goal.y, " and cLoc: ", self.cLoc.x, ", ", self.cLoc.y
      if math.sqrt((self.cLoc.x-self.goal.x)**2 + (self.cLoc.y-self.goal.y)**2) < self.control_radius:
        self.control_approach() ## close to goal, slow down
      else:
        self.control_cruise() ## cruising controller


  def path_timer_callback(self, event):
    rospy.loginfo("path_callback:: in")
    if self.use_naive_pid:
      return
    elif self.use_a_star_path:
      self.call_a_star_path_service(self.goal.x, self.goal.y)
    elif self.use_kinematic_path:
      self.call_kinematic_path_service(self.goal.x, self.goal.y)


  def make_rviz_marker(self, p, i, r, g, b, s,d):
    m = Marker()
    m.header.frame_id = "/uav0/world"
    m.header.stamp    = rospy.get_rostime()
    m.ns = "robot"
    m.id = i
    m.type = 2 # sphere
    m.action = 0
    m.pose.position = Point(p.x, p.y, p.z)
    m.pose.orientation.x = 0
    m.pose.orientation.y = 0
    m.pose.orientation.z = 0
    m.pose.orientation.w = 1.0
    m.scale.x = s
    m.scale.y = s
    m.scale.z = s

    m.color.r = r
    m.color.g = g
    m.color.b = b
    m.color.a = 1.0

    m.lifetime = rospy.Duration(d)
    self.pub_marker.publish(m)


  def call_a_star_path_service(self, gx, gy):
    try:
      #rospy.logwarn("call_a_star_path_service:: in: carrot: %i, %i", gx, gy)
      #rospy.logwarn("call_a_star_path_service:: in: start: %i, %i", self.cLoc.x, self.cLoc.y)
      resp = self.a_star_client(round(self.cLoc.x), round(self.cLoc.y), gx, gy, self.map_num)
      #rospy.logwarn("call_a_star_path_service:: out with path len %i", len(resp.xs))
      if resp.success:
        self.path = []
        for i in range(0, len(resp.xs)):
          t = 0.0
          if i > 0:
            t = math.atan2(resp.ys[i]-resp.ys[i-1], resp.xs[i]-resp.xs[i-1])
          else:
            t = math.atan2(resp.ys[i]-self.cLoc.x, resp.xs[i]-self.cLoc.y)
          # x,y,z,t,fs,ps,zs,ts
          if i+1 < len(resp.xs):
            a = State(resp.xs[i], resp.ys[i], self.goal.z, t, self.cruising_speed, 0.0, 0.0, 0.0);
            self.path.append(a)
          else:
            a = State(resp.xs[i], resp.ys[i], self.goal.z, t, 0.0, 0.0, 0.0, 0.0);
            self.path.append(a)
          #print "   ", resp.xs[i], ", ", resp.ys[i]
          
        return True
      else:
        rospy.logwarn("call_a_star_path_service:: fail")
        rospy.logwarn("   cLoc: %.2f, %.2f", self.cLoc.x, self.cLoc.y)
        rospy.logwarn("   carrot: %.2f, %.2f", gx, gy)
        self.path = []
        return False
    except:
      rospy.logerr("PID Controller:: Failed to call A* service")

  def call_kinematic_path_service(self, gx, gy):
    
    rospy.loginfo("requesting path")
    resp = self.kinematic_client(self.cLoc.x, self.cLoc.y, self.cLoc.t, self.cLoc.fs, gx, gy, 0.0, 0.0, self.map_num)
    if resp.success:
      self.path = []
      for i in range(0, len(resp.xs)):
        # x,y,z,t,fs,ps,zs,ts
        a = State(resp.xs[i], resp.ys[i], self.goal.z, resp.thetas[i], resp.speeds[i], 0.0, 0.0, 0.0);
        self.path.append(a)
      return True
    else:
      rospy.logwarn("PID_Controller::failed to get kinematic_path from service")
      self.path = []
      return False

  def goal_callback(self, msg):
    #rospy.loginfo("goal_callback: goal_in: %0.2f, %0.2f", msg.x,msg.y)
    if self.use_naive_pid:
      self.path = []
      self.goal.x = msg.x
      self.goal.y = msg.y
      self.goal_initialized = True
    elif self.use_a_star_path:
      if self.call_a_star_path_service(msg.x, msg.y):
        self.goal_initialized = True
        a = State(msg.x, msg.y, self.goal.z, 0.0, 0.0, 0.0, 0.0, 0.0);
        self.path.append(a)
        self.goal.x = msg.x
        self.goal.y = msg.y  
      else:
        self.goal_initialized = False
    elif self.use_kinematic_path:
      if self.call_kinematic_path_service(msg.x, msg.y):
        self.goal_initialized = True
        a = State(msg.x, msg.y, self.goal.z, 0.0, 0.0, 0.0, 0.0, 0.0);
        self.path.append(a)
        self.goal.x = msg.x
        self.goal.y = msg.y
      else:
        self.goal_initialized = False
    else:
      rospy.logerr("PID_Controller::goal_callback: NO PLANNING METHOD PROVIDED")
      self.goal_initialized = False
    
    #for i, p in enumerate(self.path):
    #  self.make_rviz_marker(p, i, 1.0,0.0,0.0,0.5,10.0)
    self.make_rviz_marker(self.goal, 99, 0.0,0.0,1.0,1.5,10.0)
    

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
      self.carrot.x = self.cLoc.x
      self.carrot.y = self.cLoc.y
      self.carrot.z = self.goal.z
      self.initialized = True

  def limit_goals(self):
    self.carrot.x = min(max(self.carrot.x, self.min_state.x), self.max_state.x)
    self.carrot.y = min(max(self.carrot.y, self.min_state.y), self.max_state.y)

  def limit_twist_out(self, pid):
    pid.fs = min(max(pid.fs, self.min_state.fs), self.max_state.fs)
    pid.ps = min(max(pid.ps, self.min_state.ps), self.max_state.ps)
    pid.zs = min(max(pid.zs, self.min_state.zs), self.max_state.zs)
    pid.ts = min(max(pid.ts, self.min_state.ts), self.max_state.ts)
    return pid

  def update_goal(self):
    # check if I have a long path
    if len(self.path) > 1 and not self.use_naive_pid:
      pp = self.path
      if math.sqrt((self.cLoc.x-self.goal.x)**2 + (self.cLoc.y-self.goal.y)**2) < self.control_radius:
        # check if I am at my goal
        self.carrot = self.goal
        return

      for i in reversed(range(0,len(pp))):
        # find the point along the path that maximizes the distance from me and is less than some distance
        if math.sqrt((self.cLoc.x-pp[i].x)**2 + (self.cLoc.y-pp[i].y)**2) < self.control_radius:
          self.carrot.x = pp[i].x
          self.carrot.y = pp[i].y
          self.carrot.z = pp[i].z
          self.carrot.t = pp[i].t
          self.carrot.fs = pp[i].fs
          self.make_rviz_marker(self.carrot, i, 0.0,0.0,1.0,2.0,0.1)
          return

  def bearing_alignment(self, a, b):
    tpi = 6.28318530718
    # theta 0 -> 2 pi
    a += tpi
    a = a % tpi

    # fix roll over
    dt = abs(a - self.cLoc.t)
    dtp = abs(a - self.cLoc.t + tpi)
    dtm = abs(a - self.cLoc.t - tpi)

    if dtp < dt:
      a += tpi
    elif dtm < dt:
      a -= tpi

    return [a,b]

  def control_cruise(self):
    if self.initialized == False:
      return

    if self.goal_initialized == False:
      self.carrot.x = self.cLoc.x
      self.carrot.y = self.cLoc.y
      self.carrot.z = self.cLoc.z
    elif len(self.path) > 1:
      self.update_goal()

    self.in_loop = True
    
    self.limit_goals()


    #self.cLoc.x = 33
    #self.cLoc.y = 81
    #self.cLoc.t = -1.88

    #print "******************** Goal *******************************"
    #self.carrot.print_state()
    #print "******************** cLoc *******************************"
    #self.cLoc.print_state()

    pi = 3.14159265359
    tpi = 6.28318530718

    # theta = 0 -> 2*pi
    self.cLoc.t += tpi
    self.cLoc.t = self.cLoc.t % tpi

    # move goal to local frame
    [g_dx, g_dy, self.carrot.t] = self.position_from_a_to_b(self.cLoc, self.carrot)
    [l_dx, l_dy] = self.global_to_local_frame(g_dx, g_dy, self.cLoc.t)
    self.global_to_local_vel()
    
    [self.carrot.t, self.cLoc.t] = self.bearing_alignment(self.carrot.t, self.cLoc.t)
    
    # get error
    err = State(l_dx,l_dy,0.0,0.0,0.0,0.0,0.0,0.0) # x,y,z,t,fs,ps,zs,ts
    err.z = self.carrot.z - self.cLoc.z
    err.t = self.carrot.t - self.cLoc.t

    err.fs = self.carrot.fs - self.cLoc.fs
    err.zs = self.carrot.zs - self.cLoc.zs
    err.ts = self.carrot.ts - self.cLoc.ts

    # put in some safety stuff
    # don't spin at goal
    if abs(err.x) + abs(err.y) < 1.0 * self.goal_tolerance:
      err.t = 0.0
      err.ts = 0.0

    #else:
    #  err.x = err.x * (1.0 - (abs(err.t) / pi/24.0)) * (1.0-abs(err.z))
    #  err.y = err.y * (1.0 - (abs(err.t) / pi/24.0)) * (1.0-abs(err.z))
    #  err.fs = err.fs * (1.0 - (abs(err.t) / pi/24.0)) * (1.0-abs(err.z))
    
    pid = State(0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0)
    pid.fs = self.pid_cruise_gains.fs * err.fs + self.pid_cruise_gains.x * err.x
    pid.ps = self.pid_cruise_gains.ps * err.ps + self.pid_cruise_gains.y * err.y
    pid.zs = self.pid_cruise_gains.zs * err.zs + self.pid_cruise_gains.z * err.z
    pid.ts = self.pid_cruise_gains.ts * err.ts + self.pid_cruise_gains.t * err.t
 
    # don't travel while at the wrong alt or pointing in the wrong direction
    if abs(err.z) >= 1.0 or abs(err.t) >= pi/12.0:
      pid.fs = 0.0
      pid.ps = 0.0

    #print "***************************** pid ***************************"
    #pid.print_state()

    pid = self.limit_twist_out(pid)
    
    # Not close to goal, do some smoothing
    alpha = [0.05, 0.5, 0.9]
    self.pid_mean.fs = self.pid_mean.fs - alpha[0]*(self.pid_mean.fs-pid.fs)
    self.pid_mean.ps = self.pid_mean.ps - alpha[1]*(self.pid_mean.ps-pid.ps)
    self.pid_mean.zs = self.pid_mean.zs - alpha[2]*(self.pid_mean.zs-pid.zs)
    self.pid_mean.ts = pid.ts

    twist = Twist()  
    twist.linear.x = self.pid_mean.fs
    twist.linear.y = self.pid_mean.ps
    twist.linear.z = self.pid_mean.zs
    twist.angular.z = self.pid_mean.ts

    #print "twist: ", twist
    self.pub_twist.publish(twist) 
    self.in_loop = False

  def control_approach(self):
    #print "in control_approach"
    if self.initialized == False:
      return

    self.in_loop = True

    #print "******************** Goal *******************************"
    #self.goal.print_state()
    #print "******************** cLoc *******************************"
    #self.cLoc.print_state()

    pi = 3.14159265359
    tpi = 6.28318530718

    # theta = 0 -> 2*pi
    self.cLoc.t += tpi
    self.cLoc.t = self.cLoc.t % tpi

    # move goal to local frame
    [g_dx, g_dy, self.goal.t] = self.position_from_a_to_b(self.cLoc, self.goal)
    [l_dx, l_dy] = self.global_to_local_frame(g_dx, g_dy, self.cLoc.t)
    self.global_to_local_vel()
    
    [self.goal.t, self.cLoc.t] = self.bearing_alignment(self.goal.t, self.cLoc.t)
    
    # get error
    err = State(l_dx,l_dy,0.0,0.0,0.0,0.0,0.0,0.0) # x,y,z,t,fs,ps,zs,ts
    err.z = self.goal.z - self.cLoc.z
    err.t = 0.0 #self.goal.t - self.cLoc.t
    err.fs = -self.cLoc.fs
    err.zs = -self.cLoc.zs
    err.ts = 0.0 #-self.cLoc.ts

    # put in some safety stuff
    # don't spin at goal
    if abs(err.x) + abs(err.y) < 0.0 * self.goal_tolerance:
      err.t = 0.0
      err.ts = 0.0

    pid = State(0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0)
    pid.zs = self.pid_approach_gains.zs * err.zs + self.pid_approach_gains.z * err.z
    pid.ts = self.pid_approach_gains.ts * err.ts + self.pid_approach_gains.t * err.t
    # don't travel while at the wrong alt or pointing in the wrong direction
    if abs(err.z) >= 1.0 or abs(err.t) >= pi/12.0:
      pid.fs = 0.0
      pid.ps = 0.0
    else:
      pid.fs = self.pid_approach_gains.fs * err.fs + self.pid_approach_gains.x * err.x
      pid.ps = self.pid_approach_gains.ps * err.ps + self.pid_approach_gains.y * err.y
    


    #print "***************************** pid ***************************"
    #pid.print_state()

    pid = self.limit_twist_out(pid)
    
    alpha = [0.2, 0.5, 0.9]
    self.pid_mean.fs = self.pid_mean.fs - alpha[0]*(self.pid_mean.fs-pid.fs)
    self.pid_mean.ps = self.pid_mean.ps - alpha[1]*(self.pid_mean.ps-pid.ps)
    self.pid_mean.zs = self.pid_mean.zs - alpha[2]*(self.pid_mean.zs-pid.zs)
    self.pid_mean.ts = pid.ts

    twist = Twist()  
    twist.linear.x = self.pid_mean.fs
    twist.linear.y = self.pid_mean.ps
    twist.linear.z = self.pid_mean.zs
    twist.angular.z = self.pid_mean.ts

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
    err = math.sqrt((self.carrot.x - self.cLoc.x)**2 + (self.carrot.y - self.cLoc.y)**2  + (self.carrot.z - self.cLoc.z)**2)
    error_yaw = abs(self.carrot.t - self.cLoc.t)
    if (err < 1.0 and error_yaw < 8.0):
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
  ai = rospy.get_param('/agent_index')
  aa = rospy.get_param('/desired_altitude')
  cs = rospy.get_param('/cruising_speed')

  rospy.loginfo("Quad PID Controller::initializing")
  rospy.loginfo(" Quad PID Controller::agent index: %i", ai)
  rospy.loginfo(" Quad PID Controller::agent altitude: %.1f", aa)
  rospy.loginfo(" Quad PID Controller::cruising speed: %.1f", cs)
  
  pid.init(ai, aa, cs)
  rospy.loginfo("Quad PID Controller::initialized")
  rospy.spin()
