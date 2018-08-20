#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys, time
import crazyflie, rospy, tf

OFFSET = 0.8

def move_relative(cf, x, y):
	real_x, real_y, real_z = cf.position() 
	print "location: ({}, {} ,{})".format(real_x, real_y, real_z)
	cf.goTo(goal=[real_x + x, real_y + y,real_z], yaw=0.0, duration=1.3, relative=False)
	time.sleep(1.3)


def run(drones):
	print "init node"
	rospy.init_node('square_relative')
	print "starting"
	cfs = []
	listener = tf.TransformListener()
	for drone_name in drones:
		cf = crazyflie.Crazyflie(drone_name, listener)
		cfs.append(cf)
		cf.setParam("commander/enHighLevel", 1)
		cf.takeoff(targetHeight=0.5, duration=2.5)
		time.sleep(2.5)
	for cf, drone_name in zip(cfs, drones):
		move_relative(cf, OFFSET, 0)
	for cf, drone_name in zip(cfs, drones):
		move_relative(cf, 0, OFFSET)
	for cf, drone_name in zip(cfs, drones):
		move_relative(cf, -OFFSET, 0)
	for cf, drone_name in zip(cfs, drones):
		move_relative(cf, 0, -OFFSET)
	for cf, drone_name in zip(cfs, drones):
		cf.land(targetHeight=0.0, duration=2.0)
		time.sleep(1.5)
		cf.stop()
	

