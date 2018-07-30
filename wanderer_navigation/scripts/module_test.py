#!/usr/bin/env python  
# import roslib
# roslib.load_manifest('learning_tf')
import rospy
import math
import tf
import numpy as np
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist


class CameraPoser():
  """
  Test /camera_pose subscriber
  """
  def __init__(self):
    # parameters
    self.x = np.inf
    self.y = np.inf
    self.alpha = 0
    self.beta = 0
    self.cmd = Twist()
    self.stop_cmd = Twist()
    self.stop_cmd.linear.x = 0
    self.stop_cmd.angular.z = 0
    # methods
    self.sub_campose = rospy.Subscriber("/camera_pose", Pose, self.cb_func)
    self.cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

  def cb_func(self, data):
    rospy.loginfo("\n$$$ camera_link pose read in $$$\n")
    self.x = data.position.x
    self.y = data.position.y
    quat = [
      data.orientation.x,
      data.orientation.y,
      data.orientation.z,
      data.orientation.w
    ]
    rpy = tf.transformations.euler_from_quaternion(quat)
    self.alpha = rpy[2] # approximate to camera_link's planar orientation
    self.beta = math.atan2(-self.x, -self.y) # angular difference from map origin to wanderer
    print("x={:.3f}, y={:.3f}, aplha={:.3f}".format(self.x, self.y, self.alpha))

  def read_pose(self):
    """
    Guide wanderer go back to map origin
    """
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
      # self.sub_campose()
      rospy.loginfo("\n===============>Look, new pose coming in...\n")
      rate.sleep()
      
  def clean_shutdown(self):
    print("\n\nTurning off the poser...")
    return True    

def main():
  rospy.init_node('read_campose')
  poser = CameraPoser()
  rospy.on_shutdown(poser.clean_shutdown)
  poser.read_pose()
  rospy.spin()
  
if __name__ == '__main__':
  #global listener
  #listener = tf.TransformListener()
  main()
