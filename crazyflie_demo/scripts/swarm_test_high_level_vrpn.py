#!/usr/bin/env python

import rospy
import crazyflie
import time
import uav_trajectory

if __name__ == '__main__':
	rospy.init_node('test_high_level')

	cf1 = crazyflie.Crazyflie("crazyflie1", "crazyflie1")

	cf1.setParam("commander/enHighLevel", 1)

	cf2 = crazyflie.Crazyflie("crazyflie2", "crazyflie2")

	cf2.setParam("commander/enHighLevel", 1)

	cf3 = crazyflie.Crazyflie("crazyflie3", "crazyflie3")

	cf3.setParam("commander/enHighLevel", 1)

	try:

		cf1.takeoff(targetHeight = 1.0, duration = 2.0)
		cf2.takeoff(targetHeight = 1.0, duration = 2.0)
		cf3.takeoff(targetHeight = 1.0, duration = 2.0)
		time.sleep(5.0)

		cf1.goTo(goal = [0.5, 0.0, 0.5], yaw=0.2, duration = 2.0, relative = False)
		cf2.goTo(goal = [0.0, 0.5, 0.5], yaw=0.2, duration = 2.0, relative = False)
		cf3.goTo(goal = [0.0, -0.5, 0.5], yaw=0.2, duration = 2.0, relative = False)
		time.sleep(3.0)

		cf2.goTo(goal = [0.5, 0.0, 0.5], yaw=0.2, duration = 2.0, relative = False)
		cf3.goTo(goal = [0.0, 0.5, 0.5], yaw=0.2, duration = 2.0, relative = False)
		cf1.goTo(goal = [0.0, -0.5, 0.5], yaw=0.2, duration = 2.0, relative = False)
		time.sleep(3.0)

		cf3.goTo(goal = [0.5, 0.0, 0.5], yaw=0.2, duration = 2.0, relative = False)
		cf1.goTo(goal = [0.0, 0.5, 0.5], yaw=0.2, duration = 2.0, relative = False)
		cf2.goTo(goal = [0.0, -0.5, 0.5], yaw=0.2, duration = 2.0, relative = False)
		time.sleep(3.0)

		cf3.goTo(goal = [0.0, 0.0, 0.9], yaw=0.2, duration = 2.0, relative = False)
		cf1.goTo(goal = [0.0, 0.0, 0.6], yaw=0.2, duration = 2.0, relative = False)
		cf2.goTo(goal = [0.0, 0.0, 0.3], yaw=0.2, duration = 2.0, relative = False)
		time.sleep(3.0)

		cf3.goTo(goal = [0.0, 0.0, 0.6], yaw=0.2, duration = 2.0, relative = False)
		cf1.goTo(goal = [0.0, 0.6, 0.6], yaw=0.2, duration = 2.0, relative = False)
		cf2.goTo(goal = [0.0, -0.6, 0.6], yaw=0.2, duration = 2.0, relative = False)
		time.sleep(3.0)

		cf3.goTo(goal = [0.0, 0.0, 0.6], yaw=0.2, duration = 2.0, relative = False)
		cf1.goTo(goal = [0.0, 0.6, 0.6], yaw=0.2, duration = 2.0, relative = False)
		cf2.goTo(goal = [0.0, -0.6, 0.6], yaw=0.2, duration = 2.0, relative = False)
		time.sleep(3.0)

		cf3.goTo(goal = [-0.6, 0.0, 0.6], yaw=0.2, duration = 2.0, relative = False)
		cf1.goTo(goal = [0.0, 0.0, 0.6], yaw=0.2, duration = 2.0, relative = False)
		cf2.goTo(goal = [0.6, 0.0, 0.6], yaw=0.2, duration = 2.0, relative = False)
		time.sleep(3.0)

		cf3.goTo(goal = [0.0, 0.5, 0.5], yaw=0.2, duration = 2.0, relative = False)
		cf1.goTo(goal = [0.0, 0.0, 0.5], yaw=0.2, duration = 2.0, relative = False)
		cf2.goTo(goal = [0.0, -0.5, 0.5], yaw=0.2, duration = 2.0, relative = False)
		time.sleep(3.0)

		cf1.land(targetHeight = 0.0, duration = 2.0)
		cf2.land(targetHeight = 0.0, duration = 2.0)
		cf3.land(targetHeight = 0.0, duration = 2.0)
		time.sleep(5.0)
	
	finally:
		cf1.stop()
		cf2.stop()
		cf3.stop()
		time.sleep(5.0)
