#!/usr/bin/python

import rospy
from geometry_msgs.msg import Twist
#from tf.msg import tfMessage
from std_msgs.msg import Float64
import math
import tf

x_increase = 0.05
z_increase = 0.05

# vo/twist transformation
def vo_to_twist(vo):
  twist = Twist()
  twist.linear.x = vo[0]
  twist.angular.z = vo[1]
  return twist

# getting yaw from quaternion
def get_yaw(quat):
  euler = tf.transformations.euler_from_quaternion(quat)
  return euler[1]

class zumy_pose_controller():
  def __init__(self):
    # initialization origin setup
    rospy.init_node('zumy_pose_controller', anonymous=True)
    rospy.loginfo("Initializing...")
    rospy.on_shutdown(self.myhook)

    self.setpoint = (0,0)
    self.origin = (0,0)
    self.current = (0,0)

    self.first_time = True

    # orientation control parameters
    self.ori_p = 0.7
    self.ori_constant_speed = 0.15
    self.ori_ctrl = True
    self.ori_tolerance = 0.02
    self.ori_error_threshold = 0.2

    self.ori_cmd = 0 #0.785398

    # distance control parameters
    self.dist_p = 1

    self.dist_cmd = 0

    # publisher/listener initializtion    
    self.pub = rospy.Publisher('odroid4/cmd_vel', Twist, queue_size=5)
    self.pub_yaw = rospy.Publisher('odroid4/yaw', Float64, queue_size=5)
    self.pub_error = rospy.Publisher('odroid4/error', Float64, queue_size=5)
    self.listener = tf.TransformListener()

  def myhook(self):
    print "shutdown time!"
    vo_cmd = (0,0)
    # try to stop zumy
    for i in range(100):
      self.pub.publish(vo_to_twist(vo_cmd))


  def measure(self):
    self.dist_cmd = math.hypot(self.setpoint[0]-self.current[0],self.setpoint[1]-self.current[1])
    self.ori_cmd = math.atan2(self.setpoint[1]-self.current[1],self.setpoint[0]-self.current[0])

  def ori_control(self, measured_yaw):
    error = self.ori_cmd-measured_yaw

    if abs(error)<self.ori_tolerance:
      return (0,0)
    if abs(error)<self.ori_error_threshold:
      w = (error/abs(error))*self.ori_constant_speed
      # w = self.ori_p_refinement * error
    else:
      w = self.ori_p * error
    return (0,w)

  def dist_control(self, d):
    v = self.dist_p * d
    return (v,0) 

  def stop(self):
    rate = rospy.Rate(20.0)

    while not rospy.is_shutdown():
        try:
            (trans,rot) = self.listener.lookupTransform('/world', '/Tracker0', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

	x = trans[0]
	y = trans[1]
	z = trans[2]

	if self.first_time:
	  self.first_time = False
	  rospy.loginfo("Getting initial pose...")
	  self.origin = (x, z) 
	  rospy.loginfo("Initial pos is (%0.4f , %0.4f)", x, z)
	  self.setpoint = (x + x_increase, z + z_increase)
	  rospy.loginfo("Setpoint is (%0.4f , %0.4f)", x + x_increase, z + z_increase)

	self.current = (x, z)
	#rospy.loginfo("Current pos is (%0.4f , %0.4f)", x, z)
	self.measure()
	#rospy.loginfo("Desire distance: %0.4f", d)
        #rospy.loginfo("Desire yaw: %0.4f", a)

	measured_yaw = get_yaw(rot)
	#rospy.loginfo("Measured yaw is %0.4f", measured_yaw)

	vo_cmd = (0,0)
	self.pub.publish(vo_to_twist(vo_cmd))

        rate.sleep()

  def get_ori_cmd(self, data): 
    self.ori_cmd = data.data

  def run(self):

    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        try:
            (trans,rot) = self.listener.lookupTransform('/world', '/Tracker0', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

	x = trans[0]
	y = trans[1]
	z = trans[2]
        measured_yaw = get_yaw(rot)

	if self.first_time:
	  self.first_time = False
	  rospy.loginfo("Getting initial pose...")
	  self.origin = (x, z) 
	  rospy.loginfo("Initial pos is (%0.4f , %0.4f)", x, z)
	  self.setpoint = (x + x_increase, z + z_increase)
	  rospy.loginfo("Setpoint is (%0.4f , %0.4f)", x + x_increase, z + z_increase)

	self.current = (x, z)
	# rospy.loginfo("Current pos is (%0.4f , %0.4f)", x, z)
	
        ################## orientation control test (over-write ori_cmd)
        ## TODO think about how to over-write in the right way (never try uncomment it)
	# self.measure()
        rospy.Subscriber("ori_cmd", Float64, self.get_ori_cmd)


	#rospy.loginfo("Desire distance: %0.4f, desire yaw: %0.4f", self.dist_cmd, self.ori_cmd)

        # switch between ori_control and dist_control
	if self.ori_ctrl:
	  #rospy.loginfo("Measured yaw is %0.4f", measured_yaw)
	  vo_cmd = self.ori_control(measured_yaw)
	  rospy.loginfo("vo_cmd is (%0.4f, %0.4f)", vo_cmd[0], vo_cmd[1])
	else:
	  vo_cmd = self.dist_control(d)

        rospy.loginfo("Error is %0.4f", self.ori_cmd-measured_yaw)

        ## TODO publish yaw_cmd, vo_cmd/linear, vo_cmd/angular, dist_cmd(maybe)
        self.pub_error.publish(self.ori_cmd-measured_yaw) # TODO get error access function
        self.pub_yaw.publish(measured_yaw)
	self.pub.publish(vo_to_twist(vo_cmd))

        rate.sleep()
  

if __name__ == '__main__':
  zpc = zumy_pose_controller()
  zpc.run()


  
  
