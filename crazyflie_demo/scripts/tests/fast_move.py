#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys, time
import crazyflie, rospy

def run(drones):
	print "init node"
	rospy.init_node('fast_move')
	print "starting"
	cfs = []
	for drone_name in drones:
		cf = crazyflie.Crazyflie(drone_name, drone_name)
		cfs.append(cf)
		cf.setParam("commander/enHighLevel", 1)
		cf.takeoff(targetHeight=0.5, duration=2.5)
		time.sleep(2.5)
	xy_pos = [[0,0], [0.3,0], [0.6,0], [0.3,0], [0,0], [-0.5,0]]
	for cf in cfs:
		for pos in xy_pos:
			print "Got to {}".format(pos)
			cf.goTo(goal=[pos[0],pos[1],0.5], yaw=0.0, duration=1, relative=False)
			time.sleep(0.2)
		cf.land(targetHeight=0.0, duration=2.0)

