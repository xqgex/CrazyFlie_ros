#!/usr/bin/env python

import rospy
import crazyflie
import time
import uav_trajectory

if __name__ == '__main__':

	rospy.init_node('test_high_level')

	cf = crazyflie.Crazyflie("crazyflie", "crazyflie")

	wand = crazyflie.Crazyflie("wand", "wand")

	cf.setParam("commander/enHighLevel", 1)

	wand.setParam("commander/enHighLevel", 2)

	cf1.takeoff(targetHeight = 1.0, duration = 2.0)
	time.sleep(5.0)
	try:
		while True:
			wandPosition = wand.wandPosition()
			cfPosition = cf.position()
			if (cfPosition.position.x > wandPosition.position.x):
				cf.goTo(goal = [wandPosition.position.x + 0.2, wandPosition.position.y, wandPosition.position.z], yaw=0.2, duration = 2.0, relative = False)
			else: 
				cf.goTo(goal = [wandPosition.position.x - 0.2, wandPosition.position.y, wandPosition.position.z], yaw=0.2, duration = 2.0, relative = False)
				
	finally:
		cf1.land(targetHeight = 0.0, duration = 2.0)
		time.sleep(5.0)
		cf1.stop()


