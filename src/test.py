#!/usr/bin/python
import rospy
from geometry_msgs.msg import Twist

import sys

def vo_to_twist(vo):
  twist = Twist()
  twist.linear.x = vo[0]
  twist.angular.z = vo[1]
  return twist

if __name__ =='__main__':

  # print("input is %s",sys.argv[1])

  rospy.init_node('test',anonymous=True)
  pub = rospy.Publisher('odroid4/cmd_vel', Twist, queue_size = 5 )
  rate = rospy.Rate(100)
  counter = 0
  w = 0
  zpc = zumy_pose_controller()
  zpc.info('yaw_dot')

  while not rospy.is_shutdown():

    if counter%400<100:
      w = 1
    elif counter%400<200:
      w = 0
    elif counter%400<300:
      w = -1
    else:
      w = 0
    vo = (0,w*0.4)
    pub.publish(vo_to_twist(vo))
    counter+=1

    rate.sleep()
    
  
