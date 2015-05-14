#!/usr/bin/python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import math
import tf
import sys

x_increase = 0.3
y_increase = 0.3

if len(sys.argv) < 2:
  robot_name = 'odroid4'
else:
  robot_name = sys.argv[1]

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
  def __init__(self):
    # initialization origin setup
    rospy.init_node('zumy_pose_controller', anonymous=True)
    rospy.loginfo("Initializing...")
    self.listener = tf.TransformListener()
 
    ## TODO need to check if the arguments are valid 
    # self.setpoint = (sys.argv[1],sys.argv[2])
    # rospy.loginfo("Setpoint is (%s,%s)", self.setpoint[0], self.setpoint[1])
    
    # pose and setpoint initialization 
    [x,y,z,measured_yaw,success] = self.tf_parser()
    rospy.loginfo("Getting initial pose...")
    self.origin = (x, y)
    self.current = (x, y) 
    rospy.loginfo("Initial pos is (%0.4f , %0.4f)", x, y)
    self.setpoint = (x + x_increase, y + y_increase)
    rospy.loginfo("Setpoint is (%0.4f , %0.4f)", x + x_increase, y + y_increase)

    # orientation control parameters
    self.ori_p = 0.6
    self.ori_constant_speed = 0.2
    self.ori_ctrl = True
    self.ori_tolerance = 0.02
    self.ori_error_threshold = 0.4
    self.ori_cmd = 0 

    # distance control parameters
    self.dist_p = 1
    self.dist_cmd = 0
    self.dist_tolerance = 0.01
    self.dist_error_threshold = 0.05
    self.direction = 0

    # publisher/listener initializtion    
    self.pub = rospy.Publisher(robot_name + '/cmd_vel', Twist, queue_size=5)
    self.pub_yaw = rospy.Publisher(robot_name + '/yaw', Float64, queue_size=5)
    self.pub_yaw_error = rospy.Publisher(robot_name + '/yaw/error', Float64, queue_size=5)
    self.pub_yaw_dot = rospy.Publisher(robot_name + '/yaw_dot',Float64, queue_size=5)

    # info variables
    self.ori_error = 0 
    self.yaw = 0

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

    # shutdown callback
    rospy.on_shutdown(self.shutdown_cb)

  def shutdown_cb(self):
    print "Shutting down Zumy..."
    vo_cmd = (0,0)
    for i in range(100):
      self.pub.publish(vo_to_twist(vo_cmd))
 
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
    self.yaw = yaw - 1.57

    # yaw_dot update
    dy = yaw - self.last_yaw
    now = rospy.get_time()
    dt = now - self.last_time
    self.yaw_dot = dy/dt
    self.last_yaw = yaw
    self.last_time = now

    # ori_error update
    self.ori_error = self.ori_cmd - yaw
    
    # current position update
    self.current = (x, y)

    # ori_cmd and dist_cmd update
    ## TODO should make a switch for each mode here (input_cmd/set_cmd)
    self.set_cmd()
    ## TODO add another subscriber for dist_cmd
    #rospy.Subscriber("ori_cmd", Float64, self.set_ori_cmd)

  def set_cmd(self):
    self.dist_cmd = math.hypot(self.setpoint[0]-self.current[0],self.setpoint[1]-self.current[1])
    self.ori_cmd = math.atan2(self.setpoint[1]-self.current[1],self.setpoint[0]-self.current[0])
    if self.ori_cmd * self.direction < 0:
      self.dist_cmd = -self.dist_cmd

  ### control functions ###
  def ori_control(self):
    error = self.ori_cmd - self.yaw
    if abs(error) < self.ori_tolerance:
      ## TODO need to make sure it is still at the point when it turns
      self.ori_ctrl=False
      self.set_info_type('ori_error','off')
      self.set_info_type('yaw','off')
      rospy.loginfo('orientation control is done!')
      self.show_info('cur_pos','once')
      self.show_info('setpoint','once')
      self.show_info('ori_cmd','once')
      self.show_info('yaw','once')

      self.set_info_type('cur_pos','on')
      self.set_info_type('setpoint','off')
      self.set_info_type('dist_cmd','on')

      self.direction = self.ori_cmd
      return (0,0)
    if abs(error) < self.ori_error_threshold:
      w = (error/abs(error)) * self.ori_constant_speed
    else:
      w = self.ori_p * error
    return (0,w)

  def dist_control(self):
    if abs(self.dist_cmd) < self.dist_tolerance:
      self.set_info_type('cur_pos','off')
      self.set_info_type('dist_cmd','off')
      return (0,0)
    if abs(self.dist_cmd) < self.dist_error_threshold:
      v = 0.2
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
      rospy.loginfo("Orientation error is %0.4f", self.ori_error)

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
	
	if self.ori_ctrl:
          self.show_info('yaw')
	  vo_cmd = self.ori_control()
	  # rospy.loginfo("vo_cmd is (%0.4f, %0.4f)", vo_cmd[0], vo_cmd[1])
	else:
	  vo_cmd = self.dist_control()

        self.show_info('ori_error')

        ## TODO publish yaw_cmd, dist_cmd(maybe)
        self.pub_yaw_error.publish(self.ori_error)
        self.pub_yaw.publish(self.yaw)
	self.pub.publish(vo_to_twist(vo_cmd))

        rate.sleep()

if __name__ == '__main__':
  zpc = zumy_pose_controller()
  zpc.run()


  
  
