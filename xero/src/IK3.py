#!/usr/bin/env python
# license removed for brevity

import serial
import sys
import rospy
import numpy as np
import math
import time
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist, Point
import kdl_parser_py.urdf
import PyKDL as kdl
import tf

servo = serial.Serial("/dev/ttyAMA0", 115200)
time.sleep(0.5)
filename = "../urdf/simple_arm.urdf"
(ok, tree) = kdl_parser_py.urdf.treeFromFile(filename)
chain = tree.getChain("base", "link7")
t = kdl.Tree(tree)
fksolverpos = kdl.ChainFkSolverPos_recursive(chain)
iksolvervel = kdl.ChainIkSolverVel_pinv(chain)
iksolverpos = kdl.ChainIkSolverPos_NR(chain, fksolverpos, iksolvervel)

class xero(object):
	def __init__(self):

		self._sub = rospy.Subscriber("cntl", Twist, self._callback)

		self.msg = Float32MultiArray()
		self.cmd = Twist()
		self.last_cmd = Twist()
		self.R = Point()
		self.L = Point()

	def _callback(self, cmd):
#		print (cmd.angular.z, self.last_cmd.angular.z)
		if cmd.angular.z != self.last_cmd.angular.z:
#		if cmd.angular.z - self.last_cmd.angular.z != 0.0:
#			print "ifif"
#			print self.cmd.angular.z
			self.last_cmd.angular.z = cmd.angular.z
			self.walk(cmd.angular.z)

	def walk(self, data):
		print data
		if data != 0.0:
			fg = 0
			int_y = -0.02
			self.R.x = 0.0
			self.R.y = 0.0
			self.R.z = 0.0
			self.L.x = 0.0
			self.L.y = 0.0
			self.L.z = 0.0
			j = 0.001
			self.set_angle(1, 100, 0, int_y, 0.135)
			self.set_angle(2, 100, 0, int_y, 0.135)
			time.sleep(1)

			while(data != 0.0):
				print "a"
				if fg == 0:
					if self.R.y > 0.02:
						j = j * -1.0
						self.R.z = 0.0
						self.R.x = 0.01
						self.L.x = -0.01
					if self.R.y < -0.02:
						j = j * -1.0
						self.R.z = -0.005
						self.R.x = -0.01
						self.L.x = 0.01
						fg = 1

					self.R.y = self.R.y + j
					self.L.y = self.R.y * -1.0

				elif fg == 1:
					if self.L.y > 0.02:
						j = j * -1.0
						self.L.z = 0.0
					if self.L.y < -0.02:
						j = j * -1.0
						self.L.z = -0.005
						fg = 0

					self.L.y = self.L.y + j
					self.R.y = self.L.y * -1.0

				self.set_angle(1, 1, self.R.x * -1.0, self.R.y + int_y, 0.135 + self.R.z)
				self.ang_set(1, int(self.L.y * 20000), 10)
				self.set_angle(2, 1, self.L.x * -1.0, self.L.y + int_y, 0.135 + self.L.z)
				self.ang_set(6, int(self.R.y * 20000), 10)
				time.sleep(0.001)

				if data == 0.0:
					self.set_angle(1, 100, 0, int_y, 0.135)
					self.ang_set(1, 0, 100)
					self.set_angle(2, 100, 0, int_y, 0.135)
					self.ang_set(6, 0, 100)
					time.sleep(1)
					self.ang_set(0xFF, 0, 100)
					break

	def move_servo(self, mode):
		j = 0.001
		if self.mode == 0:
			
			

	def set_angle(self, stts, spd, x, y, z):
		init_ang = 0
		init_jnt = kdl.JntArray(6)
		init_jnt = [0, -1.0 * math.pi / 2, math.pi / 2, math.pi / 2, -1.0 * math.pi / 2, 0]

		q = tf.transformations.quaternion_from_euler(0, 0, 0)
		F2 = kdl.Frame(kdl.Rotation.Quaternion(q[0], q[1], q[2], q[3]), kdl.Vector(x, y, z))
		inv_angle = kdl.JntArray(chain.getNrOfJoints())
		iksolverpos.CartToJnt(init_jnt, F2, inv_angle)
		for i in xrange(1, 5):
			if inv_angle[i] < 0:
				inv_angle[i] = inv_angle[i] * -1.0
		for i in xrange(6):
			if inv_angle[i] > math.pi:
				while inv_angle[i] > math.pi:
					inv_angle[i] = inv_angle[i] - math.pi * 2
			if inv_angle[i] > math.pi:
				while inv_angle[i] < -math.pi:
					inv_angle[i] = inv_angle[i] + math.pi * 2

		if stts == 1:
			frame_st = 11
		if stts == 2:
			frame_st = 17
		for i in range(6):
			if i == 1:
				init_ang  =70
			else:
				init_ang = 0
			self.ang_set(i + frame_st, int(inv_angle[i] / math.pi * 1800) + init_ang, spd)

	def ang_set(self, ID, ang, speed):
		check = 0x00
		txdata = [0xFA, 0xFA, ID, 0x00, 0x1E, 0x04, 0x01, 0x00FF&ang, 0x00FF&(ang >> 8), 0x00FF&speed, 0x00FF&(speed >> 8)]
		for i in range(2, 11):
			check = check ^ txdata[i]
		txdata.append(check)
		servo.write(txdata)
		time.sleep(0.00025)

if __name__ == "__main__":
	rospy.init_node("xero_node")
	func = xero()

	try:
		rospy.spin()

	except KeyboardInterrupt:
		pass
