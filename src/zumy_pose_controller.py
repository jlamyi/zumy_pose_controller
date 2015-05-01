#!/usr/bin/python

import rospy
from geometry_msgs.msg import Twist
from tf.msg import tfMessage
#from geometry_msgs.msg import Pose
import math
import tf

def vo_to_twist(vo):
  twist = Twist()
  twist.linear.x = vo[0]
  twist.angular.z = vo[1]
  return twist

## getting yaw from quaternion
def get_yaw(quat):
  euler = tf.transformations.euler_from_quaternion(quat)
  return euler[2]

x_increase = 0.1
z_increase = 0.1

class zumy_pose_controller():
  def __init__(self):
    ## TODO initialization origin setup
    rospy.init_node('zumy_pose_controller', anonymous=True)
    rospy.loginfo("Initializing...")

    self.setpoint = (0,0)
    self.origin = (0,0)
    self.current = (0,0)
    self.dist_p = 1
    self.ori_p = 1
    self.ori_control = True
    self.first_time = True
    
    self.pub = rospy.Publisher('robot0/cmd_vel', Twist, queue_size=10)
    self.listener = tf.TransformListener()

  def measure(self):
    dist = math.hypot(self.setpoint[0]-self.current[0],self.setpoint[1]-self.current[1])
    angle = math.atan2(self.setpoint[1]-self.current[1],self.setpoint[0]-self.current[0])
    return (dist, angle)

  def orientation_control(self, a, desire_yaw):
    w = self.ori_p * (a - desire_yaw)
    ## TODO figure out what is the positive direction for w
    # what is the positive direction for w in twist??? here is left
    return (0,w)

  def distance_control(self, d):
    v = self.dist_p * d
    return (v,0) 
    
  '''def controller_callback(self, data):
    transform = data.transforms.transform
    ### y is the height in tracking tool
    x = transform.translation.x
    y = transform.translation.y
    z = transform.translation.z
    
    if self.first_time:
      self.first_time = False
      rospy.info("Getting initial pose...")
      self.origin = (x, z) 
      rospy.info("Initial pos is (%0.4f , %0.4f)", x, z)
      self.setpoint = (x + x_increase, z + z_increase)
      rospy.info("Setpoint is (%0.4f , %0.4f)", x + x_increase, z + z_increase)

    self.current = (x, z)
    rospy.info("Current pos is (%0.4f , %0.4f)", x, z)
    (d, a) = self.measure()
    rospy.loginfo("Desire distance: %0.4f, desire yaw: %0.4f", d, a)

    if self.ori_control:
      measured_yaw = get_yaw((transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w))
      rospy.loginfo("Measured yaw is %0.4f", measured_yaw)
      vo_cmd = self.orientation_control(a, measured_yaw)
    else:
      vo_cmd = self.distance_control(d)
    pub.publish(vo_to_twist(vo_cmd))'''

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

	if self.first_time:
	  self.first_time = False
	  rospy.loginfo("Getting initial pose...")
	  self.origin = (x, z) 
	  rospy.loginfo("Initial pos is (%0.4f , %0.4f)", x, z)
	  self.setpoint = (x + x_increase, z + z_increase)
	  rospy.loginfo("Setpoint is (%0.4f , %0.4f)", x + x_increase, z + z_increase)

	self.current = (x, z)
	rospy.loginfo("Current pos is (%0.4f , %0.4f)", x, z)
	(d, a) = self.measure()
	rospy.loginfo("Desire distance: %0.4f, desire yaw: %0.4f", d, a)

	if self.ori_control:
	  measured_yaw = get_yaw(rot)
	  rospy.loginfo("Measured yaw is %0.4f", measured_yaw)
	  vo_cmd = self.orientation_control(a, measured_yaw)
          rospy.loginfo("vo_cmd is (%0.4f, %0.4f)", vo_cmd[0], vo_cmd[1])
	else:
	  vo_cmd = self.distance_control(d)
	self.pub.publish(vo_to_twist(vo_cmd))

        rate.sleep()

if __name__ == '__main__':
  zpc = zumy_pose_controller()
  zpc.run()


  
  
