#!/usr/bin/env python
import signal, sys, time, os
from Simulator import crazyflie, rospy
from tests import *

CF_NAMES = ["crazyflie2","crazyflie3"]
CF_LIST = []

CF_DICT = {}

def signal_handler(sig, frame):
	global CF_DICT
	try:
		for cf_name,cf_object in CF_DICT.iteritems():
			cf_object.land()
	finally:
		print "\nCtrl+C pressed"
		sys.exit(0)

def signal_handler_old(sig, frame):
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

def get_location():
	signal.signal(signal.SIGINT, signal_handler_old)
	global cf
	rospy.init_node('the_new_gofetch')
	listener = tf.TransformListener()
	cf = crazyflie.Crazyflie(CF_NAMES[0], CF_NAMES[0])
	cf.setParam("commander/enHighLevel", 1)
	while True:
		(real_x,real_y,real_z), rot = listener.lookupTransform("/world", "/{}".format(CF_NAMES[0]), rospy.Time(0))
		print "Position ({:.2f} , {:.2f} , {:.2f})".format(real_x,real_y,real_z)
		time.sleep(1)

def main2_takeoff_and_die():
	signal.signal(signal.SIGINT, signal_handler_old)
	global cf
	rospy.init_node('the_new_gofetch')
	listener = tf.TransformListener()
	for cf_name in CF_NAMES:
		cf = crazyflie.Crazyflie(cf_name, cf_name)
		cf.setParam("commander/enHighLevel", 1)
		(real_x,real_y,real_z), rot = listener.lookupTransform("/world", "/{}".format(cf_name), rospy.Time(0))
		print "Takeoff from ({:.2f} , {:.2f} , {:.2f})".format(real_x,real_y,real_z)
		cf.takeoff(targetHeight = 0.5, duration = 2.5)
		time.sleep(2.5)
	time.sleep(120)

def main2():
	signal.signal(signal.SIGINT, signal_handler_old)
	global cf
	rospy.init_node('the_new_gofetch')
	listener = tf.TransformListener()
	cf = crazyflie.Crazyflie(CF_NAMES[0], CF_NAMES[0])
	cf.setParam("commander/enHighLevel", 1)
	(real_x,real_y,real_z), rot = listener.lookupTransform("/world", "/{}".format(CF_NAMES[0]), rospy.Time(0))
	print "Takeoff from ({:.2f} , {:.2f} , {:.2f})".format(real_x,real_y,real_z)
	cf.takeoff(targetHeight = 0.5, duration = 2.0)
	time.sleep(2.0)
	print "Going to start location"
	cf.goTo(goal = [-0.6, 0.0, 0.5], yaw=0.0, duration = 3.0, relative = False)
	time.sleep(3)
	x = -0.6
	while x <= 1.0:
		cf.goTo(goal = [x, 0.0, 0.5], yaw=0.0, duration = 2.0, relative = False)
		time.sleep(2.5)
		(real_x,real_y,real_z), rot = listener.lookupTransform("/world", "/{}".format(CF_NAMES[0]), rospy.Time(0))
		print "At ({:.2f} , {:.2f} , {:.2f}) instead of ({:.2f} , {:.2f} , {:.2f})".format(real_x,real_y,real_z,x,0.0,0.5)
		if real_z < 0.1:
			print "Drone height too low, Aborting..."
			return
		x += 0.2
	time.sleep(5)
	print "Back to (0 , 0 , 0.5)"
	cf.goTo(goal = [0.0, 0.0, 0.5], yaw=0.0, duration = 2.0, relative = False)
	time.sleep(5)
	cf.land(targetHeight = 0.0, duration = 2.0)
	time.sleep(2)
	cf.stop()
	print "Done"

def main3():
	signal.signal(signal.SIGINT, signal_handler_old)
	global CF_LIST
	rospy.init_node('the_new_gofetch')
	cf = crazyflie.Crazyflie(CF_NAMES[0], CF_NAMES[0])
	cf.setParam("commander/enHighLevel", 1)
	CF_LIST.append(cf)
	print "Starting"
	traj1 = uav_trajectory.Trajectory()
	traj1.loadcsv("takeoff.csv")
	traj2 = uav_trajectory.Trajectory()
	traj2.loadcsv("figure8.csv")
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




def main4():
	signal.signal(signal.SIGINT, signal_handler_old)
	global cf
	listener = tf.TransformListener()
	rospy.init_node('the_new_gofetch')
	cf = crazyflie.Crazyflie(CF_NAMES[0], CF_NAMES[0])
	cf.setParam("commander/enHighLevel", 1)
	print "Starting"
	cf.takeoff(targetHeight = 0.5, duration = 2.0)
	time.sleep(2.0)
	try:
		#first = True
		#forward = True
		while True:
			cf.goTo(goal = [0, 0, 0.5], yaw=0.0, duration = 2.0, relative = False)
			time.sleep(3)
			cf.goTo(goal = [0.25, 0, 0.5], yaw=0.0, duration = 2.0, relative = False)
			time.sleep(3)
			cf.goTo(goal = [0.25, -0.25, 0.5], yaw=0.0, duration = 2.0, relative = False)
			time.sleep(3)
			cf.goTo(goal = [0, -0.25, 0.5], yaw=0.0, duration = 2.0, relative = False)
			time.sleep(3)
	finally:

		cf.land(targetHeight = 0.0, duration = 1.5)
		cf.stop()
	print "Done"

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

def main():
	test = sys.argv[1] + ".py"
	valid_tests = os.listdir('./')
	if test not in valid_tests:
		print("Please enter a valid test name")
		return 0
	arguments = " ".join(sys.argv[2:])
	signal.signal(signal.SIGINT, signal_handler)
	global CF_LIST
	for drone_name in arguments:
		cf = crazyflie.Crazyflie(drone_name, drone_name)
		CF_LIST.append(cf)
	os.system(test + " " + arguments)
	return



if __name__ == '__main__':
	main()
