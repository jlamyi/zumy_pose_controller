#!/usr/bin/python

import rospy
from zumy_pose_controller import zumy_pose_controller

if __name__ == '__main__':
  zpc = zumy_pose_controller()
  zpc.set_info_type('yaw','on')
  # zpc.set_info_type('cur_pos','on')
  zpc.stop()


  
  
