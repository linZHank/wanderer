#!/usr/bin/env python  
# import roslib
# roslib.load_manifest('learning_tf')
import rospy
import math
import tf
import numpy as np
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist


# Offset two wheels' speed by adding a term to angular speed in /cmd_vel
WHEEL_OFFSET = 0. # increase right-wheel spinning

def closeHome(position):
  return np.linalg.norm(position) <= 0.2
  
class WandererDriver():
  """
  WandererDriver drives wanderer robot
  """
  def __init__(self):
    # parameters
    self.x = np.inf
    self.y = np.inf
    self.alpha = 0
    self.beta = 0
    self.turn = 0
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
    self.beta = math.atan2(self.y, self.x) # angular difference from map origin to wanderer
    print("x={:.3f}, y={:.3f}, aplha={:.3f}".format(self.x, self.y, self.alpha))
    self.turn = -math.atan2(-math.sin(self.alpha-self.beta), -math.cos(self.alpha-self.beta))
    print("turn your head {}".format(self.turn))

  def go_home(self):
    """
    Guide wanderer go back to map origin
    """
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
      #self.sub_campose()
      if not closeHome([self.x, self.y]):
        if math.fabs(self.turn) > 0.2:  # first aims at home          
          angular = self.turn + WHEEL_OFFSET #
          if angular > 0.1:
            angular = 0.1 + WHEEL_OFFSET
          elif angular < -0.1:
            angular = -0.1 + WHEEL_OFFSET
          linear = 0
        else:  # then move straight forward
          linear = math.sqrt(self.x ** 2 + self.y ** 2)
          if linear > 0.2:
            linear = 0.2
          elif linear < -0.2:
            linear = -0.2
          angular = 0 + WHEEL_OFFSET
        self.cmd.linear.x = linear
        self.cmd.angular.z = angular
        self.cmd_vel.publish(self.cmd)
      else:
        self.clean_shutdown()
        print("wanderer is home!!!")
        
      rate.sleep()

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
  main()
 
