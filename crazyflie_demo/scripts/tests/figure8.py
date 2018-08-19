#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys, time, uav_trajectory
import crazyflie, rospy

def run(drones):
	rospy.init_node('figure8')
	cfs = [crazyflie.Crazyflie(drone_name, drone_name) for drone_name in drones]
	for cf in cfs:
		cf.setParam("commander/enHighLevel", 1)
	print("starting")
	traj1 = uav_trajectory.Trajectory()
	traj1.loadcsv("./csv/takeoff.csv")
	traj2 = uav_trajectory.Trajectory()
	traj2.loadcsv("./csv/figure8.csv")
	print(traj1.duration)
	for cf in cfs:
		cf.uploadTrajectory(0, 0, traj1)
		cf.uploadTrajectory(1, len(traj1.polynomials), traj2)
		cf.startTrajectory(0, timescale=1.0)
	time.sleep(traj1.duration * 2.0)
	while True:
		for cf in cfs:
			cf.startTrajectory(1, timescale=2.0)
		time.sleep(traj2.duration * 2.0)
	for cf in cfs:
		cf.startTrajectory(0, timescale=1.0, reverse=True)
	time.sleep(traj1.duration * 1.0)
	for cf in cfs:
		cf.stop()

