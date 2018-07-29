#!/usr/bin/env python
# license removed for brevity
import rospy
import math
import tf
import geometry_msgs.msg
from geometry_msgs.msg import Pose
from std_msgs.msg import String

def tf_publisher():
  rospy.init_node("tf_pub", anonymous=True)
  listener = tf.TransformListener()
  pub = rospy.Publisher('camera_pos', Pose, queue_size=10)
  rate = rospy.Rate(10.0)
  while not rospy.is_shutdown():
    try:
      (trans,rot) = listener.lookupTransform("/map", "/camera_link", rospy.Time(0))
      pose = Pose()
      pose.position.x = trans[0]
      pose.position.y = trans[1]
      pose.position.z = trans[2]
      pose.orientation.x = rot[0]
      pose.orientation.y = rot[1]
      pose.orientation.z = rot[2]
      pose.orientation.w = rot[3]

      pub.publish(pose)      
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
      continue

    rate.sleep()
if __name__ == '__main__':
  try:
    tf_publisher()
  except rospy.ROSInterruptException:
    pass
