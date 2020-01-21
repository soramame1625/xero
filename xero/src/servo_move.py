#!/usr/bin/env python

import serial
import time
import math
import rospy
from std_msgs.msg import Float32MultiArray

#pub = rospy.Publisher('servo_ang', Float32MultiArray, queue_size=10)

servo = serial.Serial('/dev/ttyAMA0', 115200)
time.sleep(0.5)

def torque(ID, data):
    check = 0x00
    txdata = [0xFA, 0xAF, ID, 0x00, 0x24, 0x01, 0x01, data]
    for i in range(2,8):
            check = check^txdata[i]
    txdata.append(check)
    servo.write(txdata)
    time.sleep(0.00025)

def ang_set(ID, ang,speed):
    check = 0x00
    txdata = [0xFA, 0xAF, ID, 0x00, 0x1E, 0x04, 0x01, 0x00FF&ang, 0x00FF&(ang >> 8), 0x00FF&speed, 0x00FF&(speed >> 8)]
    for i in range(2,11):
            check = check^txdata[i]
    txdata.append(check)
    servo.write(txdata)
    time.sleep(0.00025)

def node(ang):
  #print
  for i in range(6): 
    print ang.data[i] / math.pi * 180
    ang_set(i+11 ,int(ang.data[i] / math.pi * 1800),100)

rospy.init_node('servo_move')
sub = rospy.Subscriber('servo_ang',Float32MultiArray,node)
rospy.spin()
servo.close
