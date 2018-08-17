#!/usr/bin/env python

import rospy
import crazyflie
import time
import tf
import numpy as np

if __name__ == '__main__':
	rospy.init_node('test_position')

	listener = tf.TransformListener()

	cf = crazyflie.Crazyflie("crazyflie1", "crazyflie1")

	cf.setParam("commander/enHighLevel", 1)

	try:
		while True:
			(x,y,z), rot = listener.lookupTransform("/world", "/crazyflie1", rospy.Time(0))
			pos = np.array((x,y,z))
			print(pos)
			print(x)
			print(y)
			print(z)
			time.sleep(2)
	finally:
		print("exit")
		cf.stop()

