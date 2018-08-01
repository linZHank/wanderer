#!/usr/bin/env python
from __future__ import print_function
import numpy as np
import math
import random
import time
import rospy
from geometry_msgs.msg import Point, Pose, Twist

from utils import generatePoint2D, bcolors
from Wanderer import Wanderer


# class WandererDriver(Wanderer):
#   """
#   Inherent from Wanderer class
#   """
#   def __init__(self):
#     Wanderer.__init__(self)


def main():
  rospy.init_node('wanderer_gohome')
  homerunner = Wanderer()
  rospy.on_shutdown(homerunner.clean_shutdown)
  homerunner.go_home()
  rospy.spin()
  
if __name__ == '__main__':
  main()
