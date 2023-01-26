#!/usr/bin/env python

import rospy
from std_msgs.msg import *
from geometry_msgs.msg import *

import os,sys
sys.path.append(os.path.join(os.path.dirname(__file__), './python_utils'))
sys.path.append(os.path.join(os.path.dirname(__file__), './python_ros_utils'))

#from python_utils import *
from python_ros_utils import *

class PSP_string:
  def __init__(self,name):
    self.pub = rospy.Publisher(name + '_state', String, queue_size=1)
    self.sub = rospy.Subscriber(name, String, self.callback)
    self.data = rospy.get_param(name, "mode1")
    self.name = name

  def callback(self, msg):
    self.data = msg.data
    rosparam.set_param(self.name, str(self.data))

  def process(self):
    self.pub.publish(String(self.data))
    return self.data

if __name__ == '__main__':

  node_name = "param_sub_pub"
  rospy.init_node(node_name)

  test = PSP(node_name + "/test_data1")

  r = rospy.Rate(10)
  while not rospy.is_shutdown():
    print(test.process())
    r.sleep()

  '''
  - example run command
  rosrun ros_sampler psp.py _pub:=aaa _sub:=aaa
  '''

