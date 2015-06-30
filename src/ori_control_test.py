#!/usr/bin/python

import rospy
from zumy_pose_controller import zumy_pose_controller

if __name__ == '__main__':
  zpc = zumy_pose_controller()
  zpc.ori_control_test()


  
  
