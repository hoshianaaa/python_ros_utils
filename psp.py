#!/usr/bin/env python

import rospy
from std_msgs.msg import *
from geometry_msgs.msg import *

import os,sys
sys.path.append(os.path.join(os.path.dirname(__file__), './python_utils'))
sys.path.append(os.path.join(os.path.dirname(__file__), './python_ros_utils'))

from python_utils import *
from python_ros_utils import *

class PSP:
  def __init__(self,name,type="Float64",default=None):

    self.type = type

    if (self.type == "String"):
      print("psp type: String")
      self.pub = rospy.Publisher(name + '_state', String, queue_size=1)
      self.sub = rospy.Subscriber(name, String, self.callback)
      if default is not None:
        self.default_value = default
      else:
        self.default_value = "mode1"
    else:
      print("psp type: Float64")
      self.pub = rospy.Publisher(name + '_state', Float64, queue_size=1)
      self.sub = rospy.Subscriber(name, Float64, self.callback)
      if default is not None:
        self.default_value = default
      else:
        self.default_value = 0.0

    self.data = rospy.get_param(name, self.default_value)
    self.name = name

  def callback(self, msg):
    self.data = msg.data
    rosparam.set_param(self.name, str(self.data))

  def process(self):
    if (self.type == "String"):
      self.pub.publish(String(self.data))
    else:
      self.pub.publish(Float64(self.data))

    return self.data

class PSP_limit(PSP):
  def callback(self, msg):
    if msg.data > 0:
      self.data = msg.data
      rosparam.set_param(self.name, str(self.data))

class PSP_mode(PSP):
  def __init__(self, name, mode_list):
    super().__init__(name, "String")
    self.mode_list = mode_list
    if self.data in self.mode_list:
      pass
    else:
      self.data = self.mode_list[0]

  def callback(self, msg):
    if msg.data in self.mode_list:
      self.data = msg.data
      rosparam.set_param(self.name, str(self.data))

if __name__ == '__main__':

  node_name = "param_sub_pub"
  rospy.init_node(node_name)

  test = PSP_limit(node_name + "/param1")
#  test_string = PSP(node_name + "/mode", "String")
  psp_mode = PSP_mode(node_name + "/mode", ["mode1","mode2","mode3"])

  r = rospy.Rate(10)
  while not rospy.is_shutdown():
    print("param1:", test.process())
    print("mode:", psp_mode.process())
    r.sleep()

  '''
  - example run command
  rosrun ros_sampler psp.py _pub:=aaa _sub:=aaa
  '''

