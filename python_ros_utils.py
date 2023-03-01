import rospy

# core messages
from std_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *

import tf
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3

def euler_to_quaternion(euler):
    """Convert Euler Angles to Quaternion

    euler: geometry_msgs/Vector3
    quaternion: geometry_msgs/Quaternion
    """
    q = tf.transformations.quaternion_from_euler(euler.x, euler.y, euler.z)
    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

def quaternion_to_euler(quaternion):
    """Convert Quaternion to Euler Angles

    quarternion: geometry_msgs/Quaternion
    euler: geometry_msgs/Vector3
    """
    e = tf.transformations.euler_from_quaternion((quaternion.x, quaternion.y, quaternion.z, quaternion.w))
    return Vector3(x=e[0], y=e[1], z=e[2])

# rosparam
# usage: https://qiita.com/honeytrap15/items/550c757f2964b575883c
import rosparam

def now():
  return rospy.get_time()

class Timer:
  def __init__(self):
    self.start = now()
    self.last = now()

  def st(self):
    return now() - self.start

  def dt(self):
    n = now()
    ret = n - self.last
    self.last = n
    return ret

# 0.1151266098022461 ros_param_server load param time
def init(node_name, sleep=0.2):
  rospy.init_node(node_name)
  rospy.sleep(sleep)

if __name__ == '__main__':
  rospy.init_node("test")
  timer = Timer()
  r = rospy.Rate(10)

  rosparam.set_param("/aaa","1")
  get_param = rosparam.get_param("/aaa")
  print("get param:", get_param)

  while not rospy.is_shutdown():
    print("start diff:", timer.st())
    print("last diff:", timer.dt())
    r.sleep()
