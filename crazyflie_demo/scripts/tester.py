#!/usr/bin/env python
# -*- coding: utf-8 -*-
import signal, sys, time, os
import crazyflie, rospy, uav_trajectory
from tests import *

CF_LIST = []

def signal_handler(sig, frame):
	try:
		stopAllCrazyFlies()
	finally:
		print "\nCtrl+C pressed"
		sys.exit(0)

def stopAllCrazyFlies():
	global CF_LIST
	for cf in [cfs for cfs in CF_LIST if cfs != None]: # Foreach CrazyFlie
		cf.land(targetHeight = 0.0, duration = 1.5)
		print "{} forced landing".format(cf)
		time.sleep(1.5)
		cf.stop()

def main():
	signal.signal(signal.SIGINT, signal_handler)
	global CF_LIST
	test = sys.argv[1]
	valid_tests = os.listdir('./tests')
	if sys.argv[1]+".py" not in valid_tests:
		print("Please enter a valid test name")
		return 0
	arguments = sys.argv[2:]
	for drone_name in arguments:
		print "Create object for drone_name".format(drone_name)
		cf = crazyflie.Crazyflie(drone_name, drone_name)
		CF_LIST.append(cf)
	globals()[test].run(arguments)

if __name__ == '__main__':
	main()
