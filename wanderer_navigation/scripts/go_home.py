#!/usr/bin/env python  
# import roslib
# roslib.load_manifest('learning_tf')
import rospy
import math
import tf
import numpy as np
import geometry_msgs.msg

def closeHome(position):
  return np.linalg.norm(position) <= 0.1
  
class WandererDriver():
  """
  WandererDriver drives wanderer robot
  """
  def __init__(self):
    # parameters
    self.x = 1
    self.y = 1
    self.alpha = 0
    self.beta = 0
    self.posListener = tf.TransformListener()
    self.cmd = geometry_msgs.msg.Twist()
    self.stop_cmd = geometry_msgs.msg.Twist()
    self.stop_cmd.linear.x = 0
    self.stop_cmd.angular.z = 0
    # methods
    self.cmd_vel = rospy.Publisher("/cmd_vel", geometry_msgs.msg.Twist, queue_size=1)

  def go_home(self):
    """
    Guide wanderer go back to map origin
    """
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
      # trans = [0,10,100] # debug
      # rot = [1, 2, 3, 4] # debug
      try:
        (trans,quat) = self.posListener.lookupTransform("/map", "/camera_link", rospy.Time(0))
        # (trans,rot) = listener.lookupTransform('/map', '/camera_link', rospy.Time(0))
        # print(trans, rot)
      except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        continue
      self.x = trans[0]
      self.y = trans[1]
      rpy = tf.transformations.euler_from_quaternion(quat)
      self.alpha = rpy[2] # approximate to camera_link's planar orientation
      self.beta = math.atan2(-self.x, -self.y) # angular difference from map origin to wanderer
      print("x={:.3f}, y={:.3f}, aplha={:.3f}".format(self.x, self.y, self.alpha))
      if not closeHome([self.x, self.y]):
        # compute control command
        angular = self.beta - self.alpha # 
        linear = math.sqrt(self.x ** 2 + self.y ** 2)
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
  homerunner.go_home()
  rospy.spin()
  
if __name__ == '__main__':
  #global listener
  #listener = tf.TransformListener()
  main()
