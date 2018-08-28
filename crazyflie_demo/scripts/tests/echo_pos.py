#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys, time
import crazyflie, led, rospy, tf

def run(drones):
	print "init node"
	rospy.init_node('land')
	print "starting"
	listener = tf.TransformListener()
	cfs = []
	leds = []
	for drone_name in drones:
		if drone_name[:9] == "crazyflie":
			cf = crazyflie.Crazyflie(drone_name, listener)
			cfs.append(cf)
			cf.setParam("commander/enHighLevel", 1)
		elif drone_name[:3] == "led":
			ld = led.Led(drone_name, 0, 0)
			leds.append(ld)
	while True:
		pos_list = []
		for cf in cfs:
			pos_list.append("{} is at {}".format(cf.prefix, cf.position()))
		for ld in leds:
			pos_list.append("{} is at {}".format(ld.prefix, ld.getPosition()))
		print "\n".join(pos_list)
