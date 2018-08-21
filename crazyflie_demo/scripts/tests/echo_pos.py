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
		cf = crazyflie.Crazyflie(drone_name, listener)
		cfs.append(cf)
		cf.setParam("commander/enHighLevel", 1)
	while True:
		pos_list = []
		for cf in cfs:
			pos_list.append("{} is at {}".format(cf.prefix, cf.position()))
		print "\n".join(pos_list)
