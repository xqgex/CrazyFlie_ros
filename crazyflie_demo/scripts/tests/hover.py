#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys, time
import crazyflie, rospy, tf

def run(drones):
	print "init node"
	rospy.init_node('hover')
	print "starting"
	listener = tf.TransformListener()
	cfs = []
	for drone_name in drones:
		cf = crazyflie.Crazyflie(drone_name, listener)
		cfs.append(cf)
		cf.setParam("commander/enHighLevel", 1)
		(real_x, real_y, real_z), rot = listener.lookupTransform("/world", "/{}".format(drone_name), rospy.Time(0))
		print "Takeoff from ({:.2f} , {:.2f} , {:.2f})".format(real_x, real_y, real_z)
		cf.takeoff(targetHeight=0.5, duration=2.5)
		time.sleep(2.5)
	time.sleep(120)
	for cf in cfs:
		cf.land(targetHeight=0.0, duration=2.0)

