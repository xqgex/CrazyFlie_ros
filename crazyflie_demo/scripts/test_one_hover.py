import math, signal, sys, time
import crazyflie, rospy, tf, uav_trajectory
import numpy as np
#import keyboard
from crazyflie_driver.srv import *
from crazyflie_driver.msg import TrajectoryPolynomialPiece
import argparse

SLEEP_TIME = 0.5
CF_LIST = []

def signal_handler(sig, frame):
	try:
		stopAllCrazyFlies()
	finally:
		print "\nCtrl+C pressed"
		sys.exit(0)

def stopAllCrazyFlies():
	global CF_LIST
	for cf in [cfs for cfs in CF_LIST if cfs != None]:
		cf.land(targetHeight = 0.0, duration = 1.5)
		print "{} forced landing".format(cf)
		time.sleep(1.5)
		cf.stop()

def hover(drone_name):
	signal.signal(signal.SIGINT, signal_handler)
	global CF_LIST
	rospy.init_node('the_new_gofetch')
	cf = crazyflie.Crazyflie(drone_name, drone_name)
	cf.setParam("commander/enHighLevel", 1)
	CF_LIST.append(cf)
	print "Starting"
	traj1 = uav_trajectory.Trajectory()
	traj1.loadcsv("takeoff.csv")
	cf.uploadTrajectory(0, 0, traj1)
	cf.startTrajectory(0, timescale=1.0)
	print "Taking off"
	time.sleep(traj1.duration * 2.0)
	cf.goTo(goal = [0.0, 0.0, 0.5], yaw=0.0, duration = 2.0, relative = False)
	time.sleep(2.05)
	while True:
		cf.goTo(goal = [0.0, 0.0, 0.5], yaw=0.0, duration = 0.4, relative = False)
		time.sleep(SLEEP_TIME)
	cf.goTo(goal = [0.0, 0.0, 0.0], yaw=0.0, duration = 2.0, relative = False)
	time.sleep(2.05)

if __name__ == '__main__':
	try:
		parser = argparse.ArgumentParser()
		parser.add_argument("drone_name")
		args = parser.parse_args()
		hover(args.drone_name)
	except Exception as e:
		print e
