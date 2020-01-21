#!/usr/bin/env python
# license removed for brevity

import sys
import rospy
import numpy as np
import math
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist

import kdl_parser_py.urdf
import PyKDL as kdl
import tf

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

def set_angle(ja,x,y,z):
    q = tf.transformations.quaternion_from_euler(0,0,0)
    F2 = kdl.Frame(kdl.Rotation.Quaternion(q[0],q[1],q[2],q[3]),kdl.Vector(x,y,z))
    q_solved=kdl.JntArray(chain.getNrOfJoints())
    iksolverpos.CartToJnt(ja,F2,q_solved)
    #print q_solved
    return q_solved

def node(lpos):

    fg = 0
    x = lpos.angular.x
    y = lpos.angular.y
    z = lpos.angular.z
    init_jnt = kdl.JntArray(6)
    init_jnt[0] = 0
    init_jnt[1] = -math.pi/2
    init_jnt[2] = math.pi/2 
    init_jnt[3] = math.pi/2
    init_jnt[4] = -math.pi/2
    init_jnt[5] = 0

    inv_angle = set_angle(init_jnt,x,y,z)
    #msg = Int32MultiArray()
    for i in xrange(6):
        if inv_angle[i] < 0:
           inv_angle[i] = inv_angle[i] * -1
        if inv_angle[i]>math.pi:
          while inv_angle[i]>math.pi:
             inv_angle[i]=inv_angle[i]-math.pi*2
    msg.data=[]
    for i in range(6):
       msg.data.append(inv_angle[i])
    rospy.loginfo(msg)
    pub.publish(msg)
    #rate.sleep()

rospy.init_node('IK')
sub = rospy.Subscriber('leg_pos',Twist,node)
rospy.spin()
