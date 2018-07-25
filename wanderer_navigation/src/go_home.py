#!/usr/bin/env python  
# import roslib
# roslib.load_manifest('learning_tf')
import rospy
import math
import tf
import numpy as np
import geometry_msgs.msg

class WandererDriver():
  """
  WandererDriver drives wanderer robot
  """
  def __init__(self):
    # parameters
    self.pos_x = inf
    self.pos_y = inf
    self.posListener = tf.TransformListener()
    self.cmd = geometry_msgs.msg.Twist()
    self.stop_cmd = geometry_msgs.msg.Twist()
    self.stop_cmd.linear.x = 0
    self.stop_cmd.angular.z = 0
    # methods
    self.cmd_vel = rospy.Publisher("/cmd_vel", geometry_msgs.msg.Twist, queue_size=1)

  def goHome(self):
    """
    Guide wanderer go back to map origin
    """
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
      (trans,rot) = self.posListener.lookupTransform("map", "camera_link", rospy.Time(0))
      print("wanderer translate from map origin: x={}, y={}".format(trans[0], trans[1]))
      print("wanderer rotate from map(quaternion)".format(rot))
      if not close2home(trans[0], trans[1]):
        # compute control command
        angular = math.atan2(trans[1], trans[0])
        linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
        self.cmd.linear.x = linear
        self.cmd.angular.z = angular
        self.cmd_vel.publish(self.cmd)
      else:
        self.clean_shutdown()
        print("wanderer is home!!!")

  def clean_shutdown(self):
    print("\n\nTurning off the wanderer...")
    self.cmd_vel.publish(self.stop_cmd)
    return True    

def main():
  rospy.init_node('wanderer_gohome')
  homerunner = WandererDriver()
  rospy.on_shutdown(homerunner.clean_shutdown)
  homerunner.goHome()
  rospy.spin()
  
if __name__ == '__main__':
  main()
