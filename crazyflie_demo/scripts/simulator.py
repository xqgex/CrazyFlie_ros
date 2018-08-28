#!/usr/bin/env python
# -*- coding: utf-8 -*-

class internal_listener(object):
	def lookupTransform(self, str_0, str_1, int_0):
		return [[1,1,1],0]

class crazyflie(object):
	@staticmethod
	def Crazyflie(str_0, str_1):
		return crazyflie()
	def setParam(self, str_0, int_0):
		return
	def takeoff(self, targetHeight=0, duration=0):
		return
	def land(self, targetHeight=0, duration=0):
		return
	def goTo(self, goal=[0,0,0], yaw=0.0, duration=0, relative=False):
		return
	def stop(self):
		return
	def position(self):
		return [1,1,1]

class rospy():
	@staticmethod
	def init_node(str_0):
		return
	@staticmethod
	def Time(int_0):
		return 0

class tf(object):
	@staticmethod
	def TransformListener():
		return internal_listener()

if __name__ == "__main__":
	print "This is not the way to do it..."

