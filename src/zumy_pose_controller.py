#!/usr/bin/python

import rospy
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float64
import math
import tf
import sys
import time

## TODO get rid of robot name
robot_name = 'odroid3'

# vo/twist transformation
def vo_to_twist(vo):
  twist = Twist()
  twist.linear.x = vo[0]
  twist.angular.z = vo[1]
  return twist

# getting yaw from quaternion
def get_yaw(quat):
  euler = tf.transformations.euler_from_quaternion(quat)
  return euler[2]

class zumy_pose_controller():
  def __init__(self, s1=None, s2=None):
    # initialization origin setup
    rospy.init_node('zumy_pose_controller', anonymous=True)
    rospy.loginfo("Initializing...")

    # state machine initialization
    self.state = 'ori_ctrl'

    # mode selection
    if s1 == None and s2 == None:
        self.manual = False
        print "Auto mode"
    else:
        self.manual = True
        print "Manual mode"

    # listener initialization
    self.listener = tf.TransformListener()

    # publisher initialization    
    self.pub = rospy.Publisher(robot_name + '/cmd_vel', Twist, queue_size=5)
    self.pub_yaw = rospy.Publisher(robot_name + '/yaw', Float64, queue_size=5)
    self.pub_yaw_error = rospy.Publisher(robot_name + '/yaw/error', Float64, queue_size=5)
    self.pub_yaw_dot = rospy.Publisher(robot_name + '/yaw_dot',Float64, queue_size=5)

    # subscriber initialization
    if not self.manual:
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.setpointCB)
     
    # pose initialization 
    [x,y,z,measured_yaw,success] = self.tf_parser()
    while not success:
        print "Waiting for OptiTrack localization..."
        [x,y,z,measured_yaw,success] = self.tf_parser()

    self.current = (x, y) 
    rospy.loginfo("Initial pos is (%0.4f , %0.4f)", x, y)

    # set point initialization
    if self.manual:
        self.setpoint = (s1,s2)
    else:
        self.setpoint = None 
        ## TODO if it falls into this loop, it cannot be exited by interrupt
        while self.setpoint == None:
            time.sleep(1)
            print "Waiting for set point..."

    rospy.loginfo("Initial setpoint is (%0.4f , %0.4f)", self.setpoint[0], self.setpoint[1])

    ## TODO self.origin may be useless
    self.origin = (x, y)    

    # orientation control parameters
    self.ori_p = 0.6
    self.ori_constant_speed_low = 0.2
    self.ori_constant_speed_high = 0.5
    self.ori_tolerance = 0.02
    self.ori_error_low_threshold = 0.4
    self.ori_error_high_threshold = self.ori_constant_speed_high/self.ori_p
    self.ori_cmd = 0 

    # distance control parameters
    self.dist_p = 1
    self.dist_constant_speed = 0.14
    self.dist_cmd = 0
    self.dist_tolerance = 0.1
    self.dist_error_threshold = 0.08
    self.direction = 0

    # info variables
    self.ori_error = 0 
    self.yaw = 0
    self.vo_cmd = (0,0)
    
    # info settings
    self.yaw_dot = 0
    self.last_yaw = 0
    self.last_time = rospy.get_time()

    self.info_yaw = False
    self.info_yaw_dot = False
    self.info_ori_error = False
    self.info_cur_pos = False
    self.info_setpoint = False
    self.info_dist_cmd = False
    self.info_ori_cmd = False

    # timer
    self.timer_lock = False
    self.start_time = rospy.get_time()

    # shutdown callback
    rospy.on_shutdown(self.shutdown_cb)

  ### callback functions ###
  def shutdown_cb(self):
    print "Shutting down Zumy..."
    vo_cmd = (0,0)
    for i in range(100):
      self.pub.publish(vo_to_twist(vo_cmd))

  def setpointCB(self, data):
      s1 = data.pose.position.x
      s2 = data.pose.position.y
      self.setpoint = (s1, s2)
      if self.state == 'stop':
          self.state = 'ori_ctrl' 
 
  ### update functions ###
  def tf_parser(self):
    try:
        (trans,rot) = self.listener.lookupTransform('/correct_world', '/Tracker0', rospy.Time(0))     
	x = trans[0]
	y = trans[1]
	z = trans[2]
	measured_yaw = get_yaw(rot)
        success = True
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        ## TODO Exception can be better/ change print statement into rospy.loginfo
        print "fail to fetch tf topic"
        success = False
	return [0,0,0,0,False]
        #continue
    return [x,y,z,measured_yaw,success]

  def update(self):
    [x,y,z,yaw,success] = self.tf_parser()

    # yaw update
    self.yaw = yaw

    # yaw_dot update
    dy = yaw - self.last_yaw
    now = rospy.get_time()
    dt = now - self.last_time
    self.yaw_dot = dy/dt
    self.last_yaw = yaw
    self.last_time = now

    # ori_error update
    self.ori_error = (self.ori_cmd - self.yaw) * 57.29578
    
    # current position update
    self.current = (x, y)

    # ori_cmd and dist_cmd update
    ## TODO should make a switch for each mode here (input_cmd/set_cmd)
    self.set_cmd()
    ## TODO add another subscriber for dist_cmd
    #rospy.Subscriber("ori_cmd", Float64, self.set_ori_cmd)

  def publish(self):
    ## TODO publish yaw_cmd, dist_cmd(maybe)

    # info publish
    self.pub_yaw_error.publish(self.ori_error)
    self.pub_yaw.publish(self.yaw)

    # output publish
    self.pub.publish(vo_to_twist(self.vo_cmd))

  def set_cmd(self):
    self.dist_cmd = math.hypot(self.setpoint[0]-self.current[0],self.setpoint[1]-self.current[1])
    self.ori_cmd = math.atan2(self.setpoint[1]-self.current[1],self.setpoint[0]-self.current[0])
    if self.ori_cmd * self.direction < 0:
      self.dist_cmd = -self.dist_cmd

  ### state machine ###
  def state_machine(self):
    if self.state == 'stop':
      self.vo_cmd = (0,0)
      rospy.loginfo('Stopped, waiting for commands...')
    if self.state == 'exit':
      self.vo_cmd = (0,0)
      rospy.loginfo('Exiting...')      
    if self.state == 'break':
      self.vo_cmd = (0,0)
      rospy.loginfo('...in the break...')
      if self.timer_lock:
        if rospy.get_time() - self.start_time > 1: # wait for 5 sec
          self.timer_lock = False
          # self.set_info_type('cur_pos','on')
          self.set_info_type('dist_cmd','on')
          rospy.loginfo('Starting distance control...')
          self.state = 'dist_ctrl'
          # self.state = "stop"
      else:
        self.start_time = rospy.get_time()
        self.timer_lock = True 
      return
    if self.state == 'ori_ctrl':
      self.vo_cmd = self.ori_control()
      rospy.loginfo("vo_cmd is (%0.4f, %0.4f)", self.vo_cmd[0], self.vo_cmd[1])
      return
    if self.state == 'dist_ctrl':
      self.vo_cmd = self.dist_control()
      rospy.loginfo("vo_cmd is (%0.4f, %0.4f)", self.vo_cmd[0], self.vo_cmd[1])
      return
    return

  def ori_control(self):
    ## TODO need to improve this to self.ori_error
    error = self.ori_cmd - self.yaw
    if abs(error) < self.ori_tolerance:
      ## TODO need to make sure it is still at the point when it turns
      self.set_info_type('ori_error','off')
      self.set_info_type('yaw','off')
      rospy.loginfo('orientation control is done!')
      self.show_info('cur_pos','once')
      self.show_info('setpoint','once')
      self.show_info('ori_cmd','once')
      self.show_info('yaw','once')
      rospy.loginfo('Break for 5 sec...')

      self.direction = self.ori_cmd

      self.state = 'break'
      return (0,0)
    if abs(error) < self.ori_error_low_threshold:
      w = (error/abs(error)) * self.ori_constant_speed_low
    elif abs(error) > self.ori_error_high_threshold:
      w = (error/abs(error)) * self.ori_constant_speed_high
    else:
      w = self.ori_p * error
    if abs(error) > 3.14:
        w = -w
    return (0,w)

  def dist_control(self):
    if abs(self.dist_cmd) < self.dist_tolerance:
      self.set_info_type('cur_pos','off')
      self.set_info_type('dist_cmd','off')
      if self.manual:
          self.state = 'exit'
      else:
          self.state = 'stop'
      return (0,0)
    if abs(self.dist_cmd) < self.dist_error_threshold:
      v = self.dist_constant_speed
    else:
      v = self.dist_p * self.dist_cmd
    return (v,0) 

  ### info system ###
  def set_info_type(self, info_type, switch):
    if switch == 'on':
      value = True
    if switch == 'off':
      value = False

    if info_type == 'yaw':
      self.info_yaw = value
    if info_type == 'yaw_dot':
      self.info_yaw_dot = value
    if info_type == 'ori_error':
      self.info_ori_error = value
    if info_type == 'cur_pos':
      self.info_cur_pos = value
    if info_type == 'setpoint':
      self.info_setpoint = value
    if info_type == 'dist_cmd':
      self.info_dist_cmd = value
    if info_type == 'ori_cmd':
      self.info_ori_cmd = value

  def show_info(self, info_type, special = ''):
    if info_type == 'cur_pos' and (self.info_cur_pos or special == 'once'):
      rospy.loginfo("Current position is (%0.4f , %0.4f)", self.current[0], self.current[1])
    if info_type == 'setpoint' and (self.info_setpoint or special == 'once'):
      rospy.loginfo("Setpoint is (%0.4f , %0.4f)", self.setpoint[0], self.setpoint[1])
    if info_type == 'ori_cmd' and (self.info_ori_cmd or special == 'once'):
      rospy.loginfo("Orientation command: %0.4f", self.ori_cmd)
    if info_type == 'dist_cmd' and (self.info_dist_cmd or special == 'once'):
      rospy.loginfo("Distance command: %0.4f", self.dist_cmd)
    if info_type == 'yaw' and (self.info_yaw or special == 'once'):
      rospy.loginfo("Yaw is %0.4f degrees", self.yaw*57.29578)
    if info_type == 'yaw_dot' and (self.info_yaw_dot or special == 'once'):
      rospy.loginfo("Yaw dot is %0.4f", self.yaw_dot)
      self.pub_yaw_dot.publish(self.yaw_dot)
    if info_type == 'ori_error' and (self.info_ori_error or special == 'once'):
      rospy.loginfo("Orientation error is %0.4f degrees", self.ori_error)

  ### application functions ###
  def stop(self):
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        self.update()
        self.show_info('yaw')
        self.show_info('cur_pos')
        vo_cmd = (0,0) 
        self.pub.publish(vo_to_twist(vo_cmd))
        rate.sleep()

  def set_ori_cmd(self, data): 
    self.ori_cmd = data.data

  def run(self):
    rate = rospy.Rate(10.0)
    self.set_info_type('ori_error','on')
    self.set_info_type('yaw','on')
    # self.set_info_type('cur_pos','on')

    while not rospy.is_shutdown():
        self.update()

        self.show_info('cur_pos')
        self.show_info('dist_cmd')
        self.show_info('setpoint')
        self.show_info('ori_error')
        self.show_info('yaw')

        self.state_machine()
        self.publish()
        
        if self.state == 'exit':
            break

        rate.sleep()

  def ori_control_test(self):
   
    counter = 0

    rate = rospy.Rate(10.0)
    self.set_info_type('ori_error','on')
    self.set_info_type('yaw','on')

    self.ori_cmd = math.atan2(self.setpoint[1],self.setpoint[0])

    while not rospy.is_shutdown():
        [x,y,z,yaw,success] = self.tf_parser()

        self.yaw = yaw
        ## TODO use ori_cmd after initialization
        self.ori_error = (self.ori_cmd - self.yaw) * 57.29578

        self.show_info('ori_error')
        self.show_info('yaw')

        self.vo_cmd = self.ori_control()
        self.publish()
        
        ## for debug, to enable it stops
        if self.vo_cmd == (0,0):
            counter = counter + 1
            if counter == 30:
                break
        rate.sleep()

if __name__ == '__main__':
  zpc = zumy_pose_controller()
  zpc.run()


  
  
