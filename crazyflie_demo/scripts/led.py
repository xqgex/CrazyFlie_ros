#!/usr/bin/env python
import rospy, tf

class Led:
	def __init__(self, name, world_size_x, world_size_y):
		self.tf = tf.TransformListener()
		self.prefix = name
		self._world_size_x = world_size_x
		self._world_size_y = world_size_y

	def getPosition(self):
		self.tf.waitForTransform("/world", "/{}".format(self.prefix), rospy.Time(0), rospy.Duration(10))
		position, quaternion = self.tf.lookupTransform("/world", "/{}".format(self.prefix), rospy.Time(0))
		return [position[0] - self._world_size_x, position[1] - self._world_size_y, position[2]]

