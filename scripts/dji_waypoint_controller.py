#!/usr/bin/env python


import rospy, math
import numpy as np
import random
import sys, termios, tty, select, os
from dji_sdk.dji_drone import DJIDrone
import dji_sdk.msg
import dji_sdk.srv
from dji_sdk.msg import GlobalPosition, TransparentTransmissionData, RCChannels
from dji_sdk.srv import Activation, VelocityControl

from custom_messages.msg import DMCTS_Travel_Goal
from custom_messages.srv import Get_A_Star_Path
from geometry_msgs.msg import Twist, Pose, Point, PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Header
from visualization_msgs.msg import Marker

class State(object):

  def __init__(self, xi, yi, zi, ti, xsi, ysi, zsi, tsi):
    self.x = xi # locs
    self.y = yi # locs
    self.z = zi # locs
    self.t = ti # theta
    self.xs = ysi # forward speed
    self.ys = xsi # perpendicular speed
    self.zs = zsi # alt vel
    self.ts = tsi # theta speed

  def print_state(self):
    rospy.loginfo("x,y,z,t: %.2f, %2.f, %.2f, %.2f", self.x, self.y, self.z, self.t)
    rospy.loginfo("xs,ys,zs,ts:  %.2f, %2.f, %.2f, %.2f", self.xs, self.ys, self.zs, self.ts)


class PID_Controller(object):
  index = 0
  agent_type = 0
  start = State(0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0) # x,y,z,t,xs,ys,zs,ts
  cLoc = State(0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0) # x,y,z,t,xs,ys,zs,ts
  carrot = State(0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0) # x,y,z,t,xs,ys,zs,ts
  goal = State(0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0) # x,y,z,t,xs,ys,zs,ts
  
  pid_global_gains =   State( 0.25, 0.25,  0.5, 200.0, 0.0, 0.0, 0.1, 10.0)
  sum_err = State(0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0)
  max_state = State( 200.0, 200.0,40.0, 50.0, 0.1, 0.1, 1.0,100.0)
  min_state = State(-200.0,-200.0, 2.0,-50.0,-0.1,-0.1,-1.0,-100.0)
  pid_mean = State(0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0)
  goal_index = -1
  
  update_rate = 100 # hz
  in_loop = False
  initialized = False
  goal_initialized = False
  path_initialized = False
  path = []

  goal_tolerance = 1.0
  control_radius = 2.5

  def init(self, drone, ai, aa, cs):
    
    self.drone = drone    
    self.drone.request_sdk_permission_control() #Request to obtain control
    rospy.loginfo("DJI_CONTROLLER::Creating DJI Services")
    self.velocity_client = rospy.ServiceProxy('/dji_sdk/velocity_control', VelocityControl)
    rospy.loginfo("DJI_CONTROLLER::Created DJI Services")

    self.max_state.xs = cs
    self.min_state.xs = -cs;
    self.max_state.ys = cs
    self.min_state.ys = -cs;
    # Initial values
    self.goal.z = aa
    self.cruising_speed = cs
    self.index = ai
    # Setup publisher
    self.pub_twist = rospy.Publisher('/uav' + str(self.index) + '/cmd_vel', Twist, queue_size=10)
    self.pub_marker = rospy.Publisher('/PID_Control/path_markers', Marker, queue_size=1)
    # Setup subscriber
    self.sub_odom = rospy.Subscriber('/global/odom', Odometry, self.odom_callback,  queue_size=1 )
    self.goal_sub = rospy.Subscriber('/dmcts_' + str(self.index) + '/travel_goal', DMCTS_Travel_Goal, self.goal_callback,  queue_size=1 )
    self.a_star_client = rospy.ServiceProxy('/dmcts_' + str(self.index) + '/costmap_bridge/a_star_path', Get_A_Star_Path)
    self.pub_path_request = rospy.Publisher('/costmap/a_star_path_request', Path, queue_size=1)
    self.sub_path_request = rospy.Subscriber('/costmap/a_star_path_response', Path, self.path_callback, queue_size=1)
    
    # 
    self.good_carrot = False
    self.path_initialized = False
    self.initialized = False
    
    # which planner do I use?
    self.use_naive_pid = False
    self.use_a_star_path = True

    # map number, ensure I am planning from the right space
    self.map_num = -1

    # basically used to see how long ago the path was updated
    self.path_valid_duration = rospy.Duration(2.0)
    self.path_update_time = rospy.Time(0.0)

    # main control loop that outputs commands to the DJI quad
    self.control_timer = rospy.Timer(rospy.Duration(0.25), self.control_timer_callback)
    # calls the costmap planner and requests a path
    self.path_timer = rospy.Timer(rospy.Duration(0.5), self.path_timer_callback)
    # updates the carrot
    self.carrot_timer = rospy.Timer(rospy.Duration(0.1), self.carrot_timer_callback)
    # activates the quad
    self.activation_timer = rospy.Timer(rospy.Duration(10.0), self.activation_timer_callback)

  def activation_timer_callback(self, event):
    self.drone.request_sdk_permission_control() #Request to obtain control

  def carrot_timer_callback(self, event):
    if self.use_naive_pid:
      self.path = []
      self.carrot.x = self.goal.x
      self.carrot.y = self.goal.y
      self.good_carrot = True
    elif self.use_a_star_path:
        self.good_carrot = self.update_carrot()

  def path_timer_callback(self, event):
    self.request_path()

  def request_path(self):
    # this updates the path using the costmap
    if self.use_naive_pid:
      self.path = []
    elif self.use_a_star_path:
      start = PoseStamped()
      start.pose.position.x = self.cLoc.x
      start.pose.position.y = self.cLoc.y
      goal = PoseStamped()
      goal.pose.position.x = self.goal.x
      goal.pose.position.y = self.goal.y
      path_out = Path()
      path_out.poses.append(start)
      path_out.poses.append(goal)
      self.pub_path_request.publish(path_out)
    else:
      rospy.logerr("PID_Controller::request_path: NO PLANNING METHOD PROVIDED")
      self.goal_initialized = False
    
    #for i, p in enumerate(self.path):
    #  self.make_rviz_marker(p, i, 1.0,0.0,0.0,0.5,10.0)
    
  def path_callback(self, path):  
    self.path_initialized = True
    self.path_update_time = rospy.Time.now()
    #rospy.loginfo("PID_Controller::request_path: path set")
    self.path = []
    for pose in path.poses:
        pp = State(pose.pose.position.x, pose.pose.position.y, self.goal.z, 0.0, 0.0, 0.0, 0.0, 0.0)
        self.path.append(pp)
    
    a = State(self.goal.x, self.goal.y, self.goal.z, 0.0, 0.0, 0.0, 0.0, 0.0)
    self.path.append(a)
    #rospy.logwarn("Got path of length: %i", len(self.path))
  
  def control_timer_callback(self, event):    
    if rospy.Time.now() - self.path_update_time > self.path_valid_duration:
        self.path_initialized = False
        rospy.logwarn("PID_Controller::Path Not updated")
    #rospy.logwarn("PID_Controller::control_timer_callback: in")
    self.control_global()

  def make_rviz_marker(self, p, i, r, g, b, s,d):
    m = Marker()
    m.header.frame_id = "/world"
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
    if random.random() < 0.1:
        return True
    
    try:
      #rospy.logwarn("PID_Controller::call_a_star_path_service:: in: start: %.2f, %.2f", self.cLoc.x, self.cLoc.y)
      #rospy.logwarn("PID_Controller::call_a_star_path_service:: in: goal: %.2f, %.2f", gx, gy)
      resp = self.a_star_client(self.cLoc.x, self.cLoc.y, float(gx), float(gy), self.map_num)
      #rospy.logwarn("call_a_star_path_service:: out with path len %i", len(resp.xs))
      if resp.success:
        self.path = []
        #path_str = []
        for i in range(0, len(resp.xs)):
          # x,y,z,t,xs,ys,zs,ts
          a = State(resp.xs[i], resp.ys[i], self.goal.z, 0.0, 0.0, 0.0, 0.0, 0.0);
          self.path.append(a)
          #path_str.append("[ " + str(resp.xs[i]) + ", " + str(resp.ys[i]) + " ], ")
          
        return True
      else:
        rospy.logwarn("call_a_star_path_service:: fail")
        rospy.logwarn("   cLoc: %.2f, %.2f", self.cLoc.x, self.cLoc.y)
        rospy.logwarn("   carrot: %.2f, %.2f", gx, gy)
        self.path = []
        return False
    except:
      rospy.logerr("PID Controller:: Failed to call A* service")

  def goal_callback(self, msg):
    #rospy.logerr("PID_Controller::goal_callback: goal_in: %0.2f, %0.2f from loc %.2f, %.2f", msg.x,msg.y, self.cLoc.x, self.cLoc.y)
    self.goal.x = msg.x
    self.goal.y = msg.y
    self.goal_initialized = True
    self.make_rviz_marker(self.goal, 99, 0.0,0.0,1.0,1.5,10.0)
    self.request_path()

  def odom_callback(self, msg):
    #rospy.logwarn("PID_Controller::odom_callback: in")
    if self.in_loop == False:
      yaw = self.quaternions_to_RPY([msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])
      self.cLoc.x = msg.pose.pose.position.x
      self.cLoc.y = msg.pose.pose.position.y
      self.cLoc.z = msg.pose.pose.position.z
      self.cLoc.t = yaw 
      self.cLoc.xs = msg.twist.twist.linear.x
      self.cLoc.ys = msg.twist.twist.linear.y
      self.cLoc.zs = msg.twist.twist.linear.z
      self.cLoc.ts = msg.twist.twist.angular.z
    
    if self.initialized == False:
      self.start.x = self.cLoc.x
      self.start.y = self.cLoc.y
      self.carrot.x = self.start.x
      self.carrot.y = self.start.y
      self.carrot.z = self.goal.z
      rospy.logwarn("PID Controller::odom callback: initialized self")
      self.initialized = True

  def limit_goals(self):
    self.carrot.x = min(max(self.carrot.x, self.min_state.x), self.max_state.x)
    self.carrot.y = min(max(self.carrot.y, self.min_state.y), self.max_state.y)

  def limit_twist_out(self, pid):
    pid.xs = min(max(pid.xs, self.min_state.xs), self.max_state.xs)
    pid.ys = min(max(pid.ys, self.min_state.ys), self.max_state.ys)
    pid.zs = min(max(pid.zs, self.min_state.zs), self.max_state.zs)
    pid.ts = min(max(pid.ts, self.min_state.ts), self.max_state.ts)
    return pid

  def update_carrot(self):
    #rospy.logerr("in update_carrot")
    # check if I have a long path
    if self.use_naive_pid:
      self.carrot = self.goal
      return True

    if len(self.path) > 1:
      #rospy.logerr("Path: %i", len(self.path))
      pp = self.path
      dd = math.sqrt((self.cLoc.x-self.goal.x)**2 + (self.cLoc.y-self.goal.y)**2)
      if dd < self.control_radius:
        # check if I am at my goal
        self.carrot = self.goal
        return True
      for p in reversed(pp):
        # find the point along the path that maximizes the distance from me and is less than some distance
        dd = math.sqrt((self.cLoc.x-p.x)**2 + (self.cLoc.y-p.y)**2)
        #rospy.logerr("cloc: %.2f %.2f and pp: %.2f, %.2f with distance %.2f and control_radius %.2f", self.cLoc.x, self.cLoc.y, p.x, p.y, dd, self.control_radius)
        if dd < self.control_radius:
          self.carrot.x = p.x
          self.carrot.y = p.y
          self.carrot.z = p.z
          self.carrot.t = p.t
          self.carrot.xs = p.xs
          self.make_rviz_marker(self.carrot, random.randint(0,10000000), 0.0,0.0,1.0,2.0,0.1)
          return True
      rospy.logerr("PID Controller::update carrot: could not update carrot")
      return False
    else:
        rospy.logerr("PID Controller::update carrot: path is NULL")
        return False

  def bearing_alignment(self, a, b):
    #print "bearing_alignment: ", a, ", ", b
    tpi = 6.28318530718
    # theta 0 -> 2 pi
    a += tpi
    a = a % tpi

    #print "a: res: ", a

    # fix roll over
    dt = abs(a - b)
    dtp = abs(a - b + tpi)
    dtm = abs(a - b - tpi)

    #print "dt: ", dt
    #print "dtp: ", dtp
    #print "dtm: ", dtm

    if dtp < dt:
      a += tpi
    elif dtm < dt:
      a -= tpi
      
    #print "a: ", a

    return [a,b]

  def control_global(self):
    # it might work to send local or global position
  
    if not self.initialized:
      rospy.logwarn("PID_Controller::control_global: Quad not initialized")
      rospy.sleep(1)
      return

    #rospy.loginfo("PID_Controller::in control_cruise")
    if not self.goal_initialized:
      #rospy.logwarn("PID_Controller::control_cruise: Goal not initialized")
      self.carrot.x = self.start.x
      self.carrot.y = self.start.y
      self.carrot.z = self.goal.z
      rospy.logwarn("PID_Controller::goal is not initialized")
    #elif len(self.path) > 1 or self.use_naive_pid:

    if not self.good_carrot or not self.path_initialized:
      ez = self.carrot.z - self.cLoc.z
      ezs = self.carrot.zs - self.cLoc.zs
      pzs = self.pid_global_gains.zs * ezs + self.pid_global_gains.z * ez
      self.velocity_client(1,0.0,0.0, pzs, 0.0)
      rospy.logwarn("PID_Controller::bad carrot")
      return

    self.in_loop = True
    #rospy.logwarn("PID_Controller::control_global: in loop")
    self.limit_goals()


    #self.cLoc.x = 33
    #self.cLoc.y = 81
    #self.cLoc.t = -1.88
    
    
    #print "******************** Goal *******************************"
    #self.goal.print_state()
    #print "******************** cLoc *******************************"
    #self.cLoc.print_state()

    pi = 3.14159265359
    tpi = 6.28318530718

    # theta = 0 -> 2*pi
    self.cLoc.t += tpi
    self.cLoc.t = self.cLoc.t % tpi

   # get error
    err = State(0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0) # x,y,z,t,xs,ys,zs,ts
    err.x = self.carrot.x - self.cLoc.x
    err.y = self.carrot.y - self.cLoc.y
    # fix roll over   
    self.carrot.t = math.atan2(err.y, err.x)+pi/2.0
    [self.carrot.t, self.cLoc.t] = self.bearing_alignment(self.carrot.t, self.cLoc.t)
    
    err.z = self.carrot.z - self.cLoc.z
    err.t = self.carrot.t - self.cLoc.t

    #print "******************** Carrot *******************************"
    #self.carrot.print_state()

    err.xs = 0.0 - self.cLoc.xs
    err.ys = 0.0 - self.cLoc.ys
    err.zs = self.carrot.zs - self.cLoc.zs
    err.ts = self.carrot.ts - self.cLoc.ts

    # put in some safety stuff
    # don't spin at goal
    if abs(err.x) + abs(err.y) < self.goal_tolerance:
      err.t = 0.0
      err.ts = 0.0

    #print "************************** error ************************************** "
    #err.print_state()
    
    
    #self.sum_err.x = min(10.0, max(self.sum_err.x + err.x,-10.0))
    #self.sum_err.y = min(10.0, max(self.sum_err.y + err.y,-10.0))
    #self.sum_err.z = min(10.0, max(self.sum_err.z + err.z,-10.0))
    #self.sum_err.t = min(10.0, max(self.sum_err.t + err.t,-10.0))
    #self.sum_err.xs = self.sum_err.xs + err.xs
    #self.sum_err.ys = self.sum_err.ys + err.ys
    #self.sum_err.zs = self.sum_err.zs + err.zs
    #self.sum_err.ts = self.sum_err.ts + err.ts
    
    pid = State(0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0)
    self.pid_global_gains.xs = 0.0
    self.pid_global_gains.ys = 0.0
    
    pid.xs = self.pid_global_gains.xs * err.xs + self.pid_global_gains.x * err.x# + 0.0 * self.sum_err.x
    pid.ys = self.pid_global_gains.xs * err.ys + self.pid_global_gains.x * err.y# + 0.0 * self.sum_err.y
    pid.zs = self.pid_global_gains.zs * err.zs + self.pid_global_gains.z * err.z# + 0.0 * self.sum_err.z
    pid.ts = 0.0*self.pid_global_gains.ts * err.ts + self.pid_global_gains.t * err.t# + 0.0 * self.sum_err.t

    #print "***************************** pid-0 ***************************"
    #pid.print_state()
    
    # don't travel while at the wrong alt or pointing in the wrong direction
    if abs(err.z) >= 1.0 or abs(err.t) >= pi/6.0:
      pid.xs = 0.0
      pid.ys = 0.0
    elif abs(err.t) >= pi/12.0:
      pid.xs += pid.xs * 0.5
      pid.ys += pid.ys * 0.5
    
    #print "***************************** pid-1 ***************************"
    #pid.print_state()

    pid = self.limit_twist_out(pid)
    
    #print "***************************** pid-2 ***************************"
    #pid.print_state()

    
    # do some smoothing
    alpha = [0.75, 0.75, 0.9]
    self.pid_mean.xs = self.pid_mean.xs - alpha[0]*(self.pid_mean.xs-pid.xs)
    self.pid_mean.ys = self.pid_mean.ys - alpha[1]*(self.pid_mean.ys-pid.ys)
    self.pid_mean.zs = self.pid_mean.zs - alpha[2]*(self.pid_mean.zs-pid.zs)
    self.pid_mean.ts = pid.ts
    
    #resp = self.velocity_client(1,-5.5*pid.ys,5.5*pid.xs, self.pid_mean.zs, 0.5*self.pid_mean.ts)    
    resp = self.velocity_client(1,-5.5*self.pid_mean.ys,5.5*self.pid_mean.xs, self.pid_mean.zs, 0.5*self.pid_mean.ts)
    if not resp:
        self.dji_active = self.activation_client()
        rospy.logwarn("DJI_CONTROLLER:: failed to send velocity command")
    twist = Twist()  
    twist.linear.x = self.pid_mean.xs
    twist.linear.y = self.pid_mean.ys
    twist.linear.z = self.pid_mean.zs
    twist.angular.z = self.pid_mean.ts#

    #print "twist: ", twist
    #self.pub_twist.publish(twist) 
    self.in_loop = False

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
    
if __name__ == "__main__":
  drone = DJIDrone()
  pid = PID_Controller()
  ai = rospy.get_param('/agent_index')
  aa = rospy.get_param('/agent_altitudes')
  cs = rospy.get_param('/cruising_speeds')
  sp = rospy.get_param('/speed_penalty')

  cs = cs[ai]
  aa = aa[ai]

  rospy.loginfo("Quad PID Controller::initializing")
  rospy.loginfo(" Quad PID Controller::agent index: %i", ai)
  rospy.loginfo(" Quad PID Controller::agent altitude: %.1f", aa)
  rospy.loginfo(" Quad PID Controller::cruising speed: %.1f", cs)
  rospy.loginfo(" Quad PID Controller::speed penalty: %.1f", sp)
    
  pid.init(drone, ai, aa, cs * sp)
  rospy.loginfo("Quad PID Controller::initialized")
  rospy.spin()
