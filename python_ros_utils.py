import rospy

# core messages
from std_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *

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
