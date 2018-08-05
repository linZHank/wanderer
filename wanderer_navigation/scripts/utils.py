"""
Useful functions and classes for wanderer
"""

from __future__ import print_function
import numpy as np
from geometry_msgs.msg import Twist


def action2Twist(action):
  """ 
  Helper function convert numpy array action into geomtry_msgs Twist
  """
  cmd_vel = Twist()
  cmd_vel.linear.x = action[0]
  cmd_vel.angular.z = action[1]
  return cmd_vel

def generatePoint2D():
  """Helper function for generate 2d point"""
  x = np.random.uniform(low=-1.5, high=1.5)
  y = np.random.uniform(low=-0.75, high=0.75)
  return np.array((x, y))

def close2Home(position):
  """Helper function determins whether wanderer is close to (0,0)
  """
  return np.linalg.norm(position) <= 0.2


class bcolors:
  """ 
  For the sake of printing in colors
  Note: do->'from __future__ import print_function' to take effect
  """
  HEADER = '\033[95m'
  OKBLUE = '\033[94m'
  OKGREEN = '\033[92m'
  WARNING = '\033[93m'
  FAIL = '\033[91m'
  ENDC = '\033[0m'
  BOLD = '\033[1m'
  UNDERLINE = '\033[4m'
