#!/usr/bin/env python
# license removed for brevity

import rospy
import math
import time
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Joy
#from std_msgs.msg import Int32
from geometry_msgs.msg import Twist

pub = rospy.Publisher('cntl', Float32MultiArray, queue_size=10)
msg = Float32MultiArray()

cntl = 0
cntl_last = 0
msg.data = [0,0]

def node(data):
    global cntl_last
    msg.data.pop(0)
    msg.data.insert(0,data.axes[4])
    msg.data.pop(1)
    msg.data.insert(1,data.axes[2])
    '''if cntl != cntl_last:
       if cntl_last == 0:
          msg.angular.z = 1
          pub.publish(msg)
       if cntl == 0:
          msg.angular.z = 0
          pub.publish(msg)'''
    cntl_last = cntl
    #pub.publish(msg)
    #print msg

rospy.init_node('Cntl')
sub = rospy.Subscriber('joy',Joy,node)
while(1):
   pub.publish(msg)
   print msg
   time.sleep(0.015)
rospy.spin()
