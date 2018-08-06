#!/usr/bin/env python
from __future__ import print_function

import numpy as np
import math
import random
import time
import rospy
import tf
from geometry_msgs.msg import Point, Pose, Twist

from utils import generatePoint2D, bcolors, close2Home

WHEEL_OFFSET = 0


class Wanderer():
  """
  Wanderer class
  """
  def __init__(self):
    # parameters
    self.x = np.inf 
    self.y = np.inf
    self.alpha = 0 # approx. planar orientation
    # self.beta = 0
    # self.turn = 0 # angular velocity control command
    self.reward = 0
    self.goal = Point()
    self.cmd = Twist()
    self.stop_cmd = Twist()
    self.stop_cmd.linear.x = 0
    self.stop_cmd.angular.z = 0
    self.reset_dur = .2 # duration for setting new goal
    self.freq = 10  # topics pub and sub rate
    self.reset_stamp = time.time()
    self.time_elapse = 0.
    # methods
    self.sub_campose = rospy.Subscriber("/camera_pose", Pose, self.campose_callback)
    self.set_goal = rospy.Publisher("/learning_goal", Point, queue_size=1)
    self.cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

  def campose_callback(self, data):
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
    # print("x={:.3f}, y={:.3f}, aplha={:.3f}".format(self.x, self.y, alpha))
    # self.beta = math.atan2(self.y, self.x) # angular difference from map origin to wanderer
    # self.turn = -math.atan2(-math.sin(self.alpha-self.beta),
    #                               -math.cos(self.alpha-self.beta))
    self.reward = -np.linalg.norm(
      np.array([self.x, self.y])-np.array([self.goal.x, self.goal.y])
    ) # negative distance

  def set_new_goal(self):
    """
    Set a new destination for wanderer
    """
    rate = rospy.Rate(self.freq)
    counter = 0
    new_goal = generatePoint2D()
    while np.linalg.norm(np.array([self.x, self.y])-new_goal) < 0.2:
      new_goal = generatePoint2D()
    self.goal.x = new_goal[0]
    self.goal.y = new_goal[1]
    self.goal.z = 0
    self.set_goal.publish(self.goal)
    print(bcolors.WARNING,
      "\n...Setting wanderer's new destination @ x={}, y={}...\n".format(new_goal[0], new_goal[1]),
      bcolors.ENDC
    )
    while not rospy.is_shutdown() and counter < self.reset_dur*self.freq:
      self.cmd_vel.publish(self.stop_cmd)
      self.reward = 0
      counter += 1
      rate.sleep()
    self.reset_stamp = time.time()

  def observe_env(self):
    """ 
    Get cart-pole state, reward and out of range flag from environment 
    """
    # For debug purpose, uncomment the following line
    # print("===> State observation: x: {0:.4f}, y: {1:.4f}, alpha: {2:.4f} \nreward: {4:.4f}".format(self.x, self.y, self.alpha, self.reward))
    return np.array([self.x, self.y, self.alpha]), self.reward

  def take_action(self):
    self.cmd_vel.publish(self.cmd)
    print(bcolors.OKGREEN, "---> Velocity command to wanderer: {}".format(self.cmd_vel), bcolors.ENDC)
        
  def go_home(self):
    """
    Guide wanderer go back to map origin
    """
    rate = rospy.Rate(10.0)
    # print("turn your head {}".format(angular_vel))
    while not rospy.is_shutdown():
      alpha = self.alpha
      beta = math.atan2(self.y, self.x)
      angular_vel = -math.atan2(-math.sin(alpha-beta),
                                -math.cos(alpha-beta))
      if not close2Home([self.x, self.y]):
        if math.fabs(angular_vel) > 0.32:  # first aims at home          
          angular_vel += WHEEL_OFFSET #
          if angular_vel > 0.1:
            angular_vel = 0.1 + WHEEL_OFFSET
          elif angular_vel < -0.1:
            angular_vel = -0.1 + WHEEL_OFFSET
          linear_vel = 0
        else:  # then move straight forward
          linear_vel = math.sqrt(self.x ** 2 + self.y ** 2)
          if linear_vel > 0.2:
            linear_vel = 0.2
          elif linear_vel < -0.2:
            linear_vel = -0.2
          angular_vel = 0 + WHEEL_OFFSET
        self.cmd.linear.x = linear_vel
        self.cmd.angular.z = angular_vel
        self.cmd_vel.publish(self.cmd)
      else:
        self.clean_shutdown()
        print("wanderer is home!!!")
      rate.sleep()

  def clean_shutdown(self):
    print("\n\nTurning off the wanderer...")
    self.cmd_vel.publish(self.stop_cmd)
    return True    
