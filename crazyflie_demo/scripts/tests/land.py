#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys, time
import crazyflie, rospy, tf

def run(drones):
	print "init node"
	rospy.init_node('land')
	print "starting"
	listener = tf.TransformListener()
	cfs = []
	for drone_name in drones:
		cf = crazyflie.Crazyflie(drone_name, drone_name)
		cfs.append(cf)
		cf.setParam("commander/enHighLevel", 1)
		cf.land(targetHeight=0.0, duration=2.0)

