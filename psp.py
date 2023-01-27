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
  def __init__(self,name,type="Float64"):

    self.type = type

    if (self.type == "String"):
      print("psp type: String")
      self.pub = rospy.Publisher(name + '_state', String, queue_size=1)
      self.sub = rospy.Subscriber(name, String, self.callback)
      self.default_value = "None"
    else:
      print("psp type: Float64")
      self.pub = rospy.Publisher(name + '_state', Float64, queue_size=1)
      self.sub = rospy.Subscriber(name, Float64, self.callback)
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

class PSP_num(PSP):
  def __init__(self, name, min_val=-1.0, max_val=1.0):
    super().__init__(name, "Float64")
    self.min_val = min_val
    self.max_val = max_val

  def callback(self, msg):
    if msg.data > self.min_val:
      if msg.data < self.max_val:
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

#  test_string = PSP(node_name + "/mode", "String")
  psp_num = PSP_num(node_name + "/param1", -2.0, 2.0)
  psp_mode = PSP_mode(node_name + "/mode", ["mode1","mode2","mode3"])

  r = rospy.Rate(10)
  while not rospy.is_shutdown():
    print("param1:", psp_num.process())
    print("mode:", psp_mode.process())
    r.sleep()

  '''
  - example run command
  rosrun ros_sampler psp.py _pub:=aaa _sub:=aaa
  '''

