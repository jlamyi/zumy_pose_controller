#!/usr/bin/python

import rospy
import tf

POS_CORRECT = [0,0,0]
# rotate -90 degrees around x-axis in rviz
ROT_CORRECT = [0.70711,0,0,-0.70711] # rotate -90 degress around z-axis in the code


if __name__ == '__main__':
  rospy.init_node('correct_tf',anonymous=True)
  rospy.loginfo('Start zumy tf corector..')
  br = tf.TransformBroadcaster()
  r = rospy.Rate(10)
  while not rospy.is_shutdown():
    br.sendTransform(POS_CORRECT,ROT_CORRECT,rospy.Time.now(),'correct_world','world')
    r.sleep() 
