#!/usr/bin/env python
# -*- coding: utf-8 -*-
import signal, sys, time, os
import crazyflie, rospy, uav_trajectory
from tests import *

CF_LIST = []
CF_NAMES = ["crazyflie2"]

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

def main3():
	signal.signal(signal.SIGINT, signal_handler)
	global CF_LIST
	rospy.init_node('the_new_gofetch')
	cf = crazyflie.Crazyflie(CF_NAMES[0], CF_NAMES[0])
	cf.setParam("commander/enHighLevel", 1)
	CF_LIST.append(cf)
	print "Starting"
	traj1 = uav_trajectory.Trajectory()
	traj1.loadcsv("./tests/takeoff.csv")
	traj2 = uav_trajectory.Trajectory()
	traj2.loadcsv("./tests/figure8.csv")
	print(traj1.duration)
	cf.uploadTrajectory(0, 0, traj1)
	cf.uploadTrajectory(1, len(traj1.polynomials), traj2)
	cf.startTrajectory(0, timescale=1.0)
	time.sleep(traj1.duration * 2.0)
	while True:
		cf.startTrajectory(1, timescale=2.0)
		time.sleep(traj2.duration * 2.0)
	cf.startTrajectory(0, timescale=1.0, reverse=True)
	time.sleep(traj1.duration * 1.0)
	cf.stop()
	print "Done"

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
