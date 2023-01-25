import rospy

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

if __name__ == '__main__':
  rospy.init_node("test")
  timer = Timer()
  r = rospy.Rate(10)
  while not rospy.is_shutdown():
    print("start diff:", timer.st())
    print("last diff:", timer.dt())
    r.sleep()
