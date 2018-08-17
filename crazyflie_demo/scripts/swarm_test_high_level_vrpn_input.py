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

	cf1.takeoff(targetHeight = 1.0, duration = 2.0)
	cf2.takeoff(targetHeight = 1.0, duration = 2.0)
	cf3.takeoff(targetHeight = 1.0, duration = 2.0)
	
	try:
		while True:
	
			goal_input = input("Enter relative movement [x,y,z]: ")
			if (type(goal_input) == list): 
				cf1.goTo(goal = goal_input, yaw=0.2, duration = 2.0, relative = True)
				cf2.goTo(goal = goal_input, yaw=0.2, duration = 2.0, relative = True)
				cf3.goTo(goal = goal_input, yaw=0.2, duration = 2.0, relative = True)
			else:
				cf1.land(targetHeight = 0.0, duration = 2.0)
				cf2.land(targetHeight = 0.0, duration = 2.0)
				cf3.land(targetHeight = 0.0, duration = 2.0)
				time.sleep(10.0)
				break
	finally:
		cf1.stop()
		cf2.stop()
		cf3.stop()
