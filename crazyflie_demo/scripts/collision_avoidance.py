#!/usr/bin/env python

import rospy
import crazyflie
import time
import uav_trajectory
import tf

if __name__ == '__main__':
	rospy.init_node('collision_avoidance')

	listener = tf.TransformListener()

	cf1 = crazyflie.Crazyflie("crazyflie1", "crazyflie1")
	cf1.setParam("commander/enHighLevel", 1)
	
	cf2 = crazyflie.Crazyflie("crazyflie3", "crazyflie3")
	cf2.setParam("commander/enHighLevel", 1)
	print("starting")
	time.sleep(10)

	cf1.takeoff(targetHeight = 1.0, duration = 2.0)
	cf2.takeoff(targetHeight = 1.0, duration = 2.0)
	time.sleep(3.0)

	(x1,y1,z1), rot = listener.lookupTransform("/world", "/crazyflie1", rospy.Time(0))
	print((x1,y1,z1))
	(x2,y2,z2), rot = listener.lookupTransform("/world", "/crazyflie3", rospy.Time(0))
	print((x2,y2,z2))


	cf1_goal = [x2, y2, z2]
	cf2_goal = [x1, y1, z1]

	try:
		cf1.goTo(goal = cf1_goal, yaw=0.2, duration = 8.0, relative = False)
		cf2.goTo(goal = cf2_goal, yaw=0.2, duration = 8.0, relative = False)

		while True:
			(x1,y1,z1), rot = listener.lookupTransform("/world", "/crazyflie1", rospy.Time(0))
			(x2,y2,z2), rot = listener.lookupTransform("/world", "/crazyflie3", rospy.Time(0))

			if (abs(x2 - x1) < 0.6): # and abs(y2 - y1) < 0.2 and  abs(z2 - z1) < 0.2):
				cf1.goTo(goal = [0.0,0.0,0.0], yaw=0.2, duration = 1.0, relative = True)
				cf2.goTo(goal = [0.0,0.0,0.0], yaw=0.2, duration = 1.0, relative = True)
				time.sleep(5.0)
				cf1.goTo(goal = [0.0,0.0,0.3], yaw=0.2, duration = 2.0, relative = True)
				time.sleep(2.0)
				cf2.goTo(goal = cf2_goal, yaw=0.2, duration = 5.0, relative = False)
				time.sleep(5.0)
				cf1.goTo(goal = [0.0,0.0,-0.3], yaw=0.2, duration = 2.0, relative = True)
				time.sleep(2.0)
				cf1.goTo(goal = cf1_goal, yaw=0.2, duration = 5.0, relative = False)
				time.sleep(5.0)
				break
		cf1.land(targetHeight = 0.0, duration = 5.0)
		cf2.land(targetHeight = 0.0, duration = 5.0)
		time.sleep(5.0)
		cf1.stop()
		cf2.stop()
		time.sleep(3.0)

	finally:
		cf1.land(targetHeight = 0.0, duration = 5.0)
		cf2.land(targetHeight = 0.0, duration = 5.0)
		time.sleep(7.0)
		cf1.stop()
		cf2.stop()
		time.sleep(3.0)


