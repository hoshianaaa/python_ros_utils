#!/usr/bin/env python

import rospy
from std_msgs.msg import *
from geometry_msgs.msg import *

import os,sys
sys.path.append(os.path.join(os.path.dirname(__file__), './python_utils'))
sys.path.append(os.path.join(os.path.dirname(__file__), './python_ros_utils'))

from python_utils import *
from python_ros_utils import *

class DataController:

  def __init__(self,topic_name,error_th=0):
    self.topic_name = topic_name
    self.sub_topic_name = topic_name + "/state"
    self.pub = generate_publisher_and_wait(self.topic_name, Float64)
    rospy.Subscriber(self.sub_topic_name, Float64, self.__state_callback)
    self.target_value = None
    self.state = None
    self.r = rospy.Rate(1000)
    self.error_th = error_th
    self.debug = 0

  def debug_on(self):
    self.debug = 1

  def wait(self):
    if self.target_value is None:
      print("[ERROR] data controller(" + self.topic_name + "): please set target value")
      return

    while not rospy.is_shutdown():
      if self.debug:
        print("[DEBUG] data controller(" + self.sub_topic_name + "): wait, state:" + str(self.state))

      if (self.state is not None):
        diff = abs(self.state - self.target_value)
        if (diff <= self.error_th):
          print("[INFO] data controller(" + self.topic_name + "): finish wait")
          break

  def send(self, value):
    self.target_value = value
    self.pub.publish(Float64(value))

  def __state_callback(self,msg):
    self.state = msg.data


if __name__ == '__main__':

  node_name = "sample"
  rospy.init_node(node_name)

  dc = DataController("data")
  dc.debug_on()
  dc.send(100)
  dc.wait()

  '''
  - example run command
  rosrun ros_sampler psp.py _pub:=aaa _sub:=aaa
  '''

# ** test command **
# rostopic pub /data
# rostopic pub /data std_msgs/Float64 "data: 10.0"
# rostopic pub /data std_msgs/Float64 "data: 100.0"


