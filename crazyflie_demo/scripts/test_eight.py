import math, signal, sys, time
import crazyflie, rospy, tf, uav_trajectory
import numpy as np
#import keyboard
from crazyflie_driver.srv import *
from crazyflie_driver.msg import TrajectoryPolynomialPiece
import argparse


def main5(name):
	
	signal.signal(signal.SIGINT, signal_handler_old)
	global cf
	rospy.init_node('the_new_gofetch')
	#cf = crazyflie.Crazyflie(CF_NAMES[0], CF_NAMES[0])
	cf = crazyflie.Crazyflie(name,name)
	cf.setParam("commander/enHighLevel", 1)
	print "Starting"
	traj1 = uav_trajectory.Trajectory()
	traj1.loadcsv("takeoff.csv")
	cf.uploadTrajectory(0, 0, traj1)
	cf.startTrajectory(0, timescale=1.0)
	print "Taking off"
	time.sleep(traj1.duration * 2.0)
	while True:
		cf.goTo(goal = [0.0, 0.0, 0.5], yaw=0.0, duration = 2.0, relative = False)
		time.sleep(2.05)
	
		
		

def main6():   #Eitan- 
	signal.signal(signal.SIGINT, signal_handler_old)
	global cf
	move=False
	rospy.init_node('the_new_gofetch')
	cf = crazyflie.Crazyflie(CF_NAMES[1], CF_NAMES[1])
	cf.setParam("commander/enHighLevel", 1)
	print "Starting"
	traj1 = uav_trajectory.Trajectory()
	traj1.loadcsv("takeoff.csv")
	cf.uploadTrajectory(0, 0, traj1)
	cf.startTrajectory(0, timescale=1.0)
	print "Taking off"
	time.sleep(traj1.duration * 2.0)
	while True:
		if not move:
			cf.goTo(goal = [0.0, 0.0, 0.5], yaw=0.0, duration = 1.0, relative = False)
			move = True
			time.sleep(1.05)
		else:
			cf.goTo(goal = [0.2, 0.0, 0.5], yaw=0.0, duration = 1.0, relative = False)
			move = False
			time.sleep(1.05)
if __name__ == '__main__':
	try:
		parser = argparse.ArgumentParser()
		parser.add_argument("name")
		args = parser.parse_args()
		#get_location()
		main5(args.name)
	except Exception as e:
		print e
