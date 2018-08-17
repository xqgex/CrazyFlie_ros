#!/usr/bin/env python

import rospy
import crazyflie
import time
import uav_trajectory
import tf
from crazyflie_driver.srv import *
from crazyflie_driver.msg import TrajectoryPolynomialPiece
import numpy as np

if __name__ == '__main__':
	rospy.init_node('wand_following')

	listener = tf.TransformListener()
	
	name = "crazyflie2"
	
	#time.sleep(20)
	
	cf = crazyflie.Crazyflie(name, name)

	cf.setParam("commander/enHighLevel", 1)

	cf.takeoff(targetHeight = 0.5, duration = 2.0)
	time.sleep(2.0)
	first = True
	(x,y,z), rot = listener.lookupTransform("/world", "/wand", rospy.Time(0))

	try:
		while True:
			(x,y,z), rot = listener.lookupTransform("/world", "/wand", rospy.Time(0))
			if first:
				cf.goToService(0, False, geometry_msgs.msg.Point(x + 0.25, y, z), 0.2, rospy.Duration.from_sec(2.0))
				time.sleep(2.0)
				first = False 
				continue
			cf.goToService(0, False, geometry_msgs.msg.Point(x + 0.25, y, z), 0.2, rospy.Duration.from_sec(1.0))
			time.sleep(0.1)
	finally:

		cf.land(targetHeight = 0.0, duration = 5.0)
		time.sleep(5.0)
		#cf.stop()
		#time.sleep(3.0)
		
