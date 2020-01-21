#!/usr/bin/env python
# license removed for brevity

import serial
import sys
import rospy
import numpy as np
import math
import time
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
import kdl_parser_py.urdf
import PyKDL as kdl
import tf

servo = serial.Serial('/dev/ttyAMA0', 115200)
time.sleep(0.5)

cmd = 0
last_cmd = 0

def ang_set(ID, ang,speed):
    check = 0x00
    txdata = [0xFA, 0xAF, ID, 0x00, 0x1E, 0x04, 0x01, 0x00FF&ang, 0x00FF&(ang >> 8), 0x00FF&speed, 0x00FF&(speed >> 8)]
    for i in range(2,11):
            check = check^txdata[i]
    txdata.append(check)
    servo.write(txdata)
    time.sleep(0.00025)

# setup kdl
filename = "../urdf/simple_arm.urdf"
(ok, tree) = kdl_parser_py.urdf.treeFromFile(filename)
chain = tree.getChain("base", "link7")
t = kdl.Tree(tree)
fksolverpos = kdl.ChainFkSolverPos_recursive(chain)
iksolvervel = kdl.ChainIkSolverVel_pinv(chain)
iksolverpos = kdl.ChainIkSolverPos_NR(chain,fksolverpos,iksolvervel)

pub = rospy.Publisher('servo_ang', Float32MultiArray, queue_size=10)
msg = Float32MultiArray()

def set_angle(stts,spd,x,y,z):
    init_ang = 0 
    init_jnt = kdl.JntArray(6)
    init_jnt[0] = 0
    init_jnt[1] = -math.pi/2
    init_jnt[2] = math.pi/2
    init_jnt[3] = math.pi/2
    init_jnt[4] = -math.pi/2
    init_jnt[5] = 0

    q = tf.transformations.quaternion_from_euler(0,0,0)
    F2 = kdl.Frame(kdl.Rotation.Quaternion(q[0],q[1],q[2],q[3]),kdl.Vector(x,y,z))
    inv_angle=kdl.JntArray(chain.getNrOfJoints())
    iksolverpos.CartToJnt(init_jnt,F2,inv_angle)
    for i in xrange(1,5):
        if inv_angle[i] < 0:
           inv_angle[i] = inv_angle[i] * -1
    for i in xrange(6):
        if inv_angle[i]>math.pi:
          while inv_angle[i]>math.pi:
             inv_angle[i]=inv_angle[i]-math.pi*2
        if inv_angle[i]>math.pi:
          while inv_angle[i]<-math.pi:
             inv_angle[i]=inv_angle[i]+math.pi*2
    if stts == 1:
      frame_st = 11
    if stts == 2:
      frame_st = 17
    for i in range(6):
       #print inv_angle[i]
       if i == 1:
         init_ang = 70
       else:
         init_ang = 0
       #print inv_angle[i] / math.pi * 180
       ang_set(i+frame_st ,int(inv_angle[i] / math.pi * 1800) + init_ang,spd)

def walk(data):
    global cmd
    #if data == 0:
    #   cmd = 1
    if data != 0:
       #cmd = 0
       fg = 0
       ini_y = -0.02
       Ry = 0
       Ly = 0
       Rz = 0
       Lz = 0
       Rx = 0
       Lx = 0
       j = 0.001
       set_angle(1,100,0,ini_y,0.135)
       set_angle(2,100,0,ini_y,0.135)
       time.sleep(1)
       #print cmd
       #print data
       while(cmd != 0):
         #print 2
         if fg == 0:
          if Ry > 0.02:
             j = j * -1
             Rz = 0.0 
             Rx = 0.01
             Lx = -0.01
          if Ry < -0.02:
             j = j * -1
             Rz = -0.005
             Rx = -0.01
             Lx = 0.01
             fg = 1          
          Ry = Ry + j
          Ly = -Ry

         elif fg == 1:
           if Ly > 0.02:
              j = j * -1
              Lz = 0.0
           if Ly < -0.02:
              j = j * -1
              Lz = -0.005
              fg = 0
           Ly = Ly + j
           Ry = -Ly
         set_angle(1,1,-Rx,Ry + ini_y,0.135 + Rz)
         ang_set(1,int(Ly * 20000), 10)
         set_angle(2,1,-Lx,Ly + ini_y,0.135 + Lz)
         ang_set(6,int(Ry * 20000), 10)
         time.sleep(0.001)

         if cmd == 0:
            set_angle(1,100,0,ini_y,0.135)
            ang_set(1,0,100)
            set_angle(2,100,0,ini_y,0.135)
            ang_set(6,0,100)
            time.sleep(1)
            ang_set(0xFF,0,100)
            break 
def node(lpos):
    global cmd
    global last_cmd
    #data = Twist()
    cmd = lpos.angular.z
    print cmd
    if cmd != last_cmd:
       last_cmd = cmd
       print cmd
       walk(cmd) 

rospy.init_node('IK')
sub = rospy.Subscriber('cntl',Twist,node)
rospy.spin()
