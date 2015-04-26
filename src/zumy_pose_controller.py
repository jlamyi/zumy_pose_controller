#!/usr/bin/env/ python
import rospy
from geometry_msgs.msg import Twist
#from geometry_msgs.msg import Pose
import math

def vo_to_twist(vo):
  twist = Twist()
  twist.linear.x = vo[0]
  twist.angular.z = vo[1]
  return twist

class zumy_pose_controller():
  def __init__(self):
    self.setpoint = (5,5)
    self.origin = (0,0)
    self.current = (0,0)
    self.dist_p = 1
    self.ori_p = 1
    self.ori_control = True
    self.pub = rospy.Publisher('robot0/cmd_vel', Twist, queue_size=10)

  def measure(self):
    dist = math.hypot(self.setpoint[0]-self.current[0],self.setpoint[1]-self.current[1])
    angle = math.atan2(self.setpoint[1]-self.current[1],self.setpoint[0]-self.current[0])
    return (dist, angle)

  def orientation_control(self, a, data.current_angel):
    w = self.ori_p * (a - data.current_angel)
    # what is the positive direction for w in twist??? here is left
    return (0,w)

  def distance_control(self, d):
    v = self.dist_p * d
    return (v,0) 
    
  def controller_callback(self,data):
    self.current = (data.pose.x, data.pose.y)
    (d, a) = self.measure()
    if self.ori_control:
      vo_cmd = self.orientation_control(a, data.current_angel)
    else:
      vo_cmd = self.distance_control(d)
    pub.publish(vo_to_twist(vo_cmd))

  def run():
    rospy.init_node('zumy_pose_controller', anonymous=True)
    rospy.Subscriber("xxx",xxx,controller_callback)
    rospy.spin()

if __name__ == '__main__':
  zpc = zumy_pose_controller()
  zpc.run()


  
  
