#!/usr/bin/python

import rospy, sys
from zumy_pose_controller import zumy_pose_controller

## TODO need to check if the arguments are valid
## TODO need take off robot_name 
if len(sys.argv) < 2:
  robot_name = 'odroid3'
  s1 = 0
  s2 = 0
else:
  robot_name = sys.argv[1]
  s1 = float(sys.argv[2])
  s2 = float(sys.argv[3])

if __name__ == '__main__':
  zpc = zumy_pose_controller(s1,s2)
  zpc.run()


  
  
