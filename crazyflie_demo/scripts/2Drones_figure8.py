#!/usr/bin/env python
import signal, sys, time
import rospy
import crazyflie
import uav_trajectory

cf = None
cf2 = None
DRONE_NAME = "crazyflie2"
DRONE_NAME2 = "crazyflie3"

def signal_handler(sig, frame):
	global cf
	global cf2
	try:
		cf.land(targetHeight = 0.0, duration = 1.5)
		cf.stop()
	finally:
		print "Killed {}".format(DRONE_NAME)
	try:
		cf2.land(targetHeight = 0.0, duration = 1.5)  Doesn't land him
		cf2.stop() Doesn't stop him
	finally:
		print "Killed {}".format(DRONE_NAME2)
	print "Ctrl+C pressed"
	sys.exit(0)

def main():	
	signal.signal(signal.SIGINT, signal_handler)
	global cf
	global cf2
	rospy.init_node('test_high_level')
	cf = crazyflie.Crazyflie(DRONE_NAME, DRONE_NAME)
	cf2= crazyflie.Crazyflie(DRONE_NAME2, DRONE_NAME2)
	cf.setParam("commander/enHighLevel", 1)
	cf2.setParam("commander/enHighLevel", 1)
	print("starting")
	traj1 = uav_trajectory.Trajectory()
	traj1.loadcsv("takeoff.csv")
	traj2 = uav_trajectory.Trajectory()
	traj2.loadcsv("figure8.csv")
	print(traj1.duration)
	cf.uploadTrajectory(0, 0, traj1)
	cf.uploadTrajectory(1, len(traj1.polynomials), traj2)
	cf.startTrajectory(0, timescale=1.0)
	cf2.uploadTrajectory(0, 0, traj1)
	cf2.uploadTrajectory(1, len(traj1.polynomials), traj2)
	cf2.startTrajectory(0, timescale=1.0)
	time.sleep(traj1.duration * 2.0)
	while 1:
		cf.startTrajectory(1, timescale=2.0)
		cf2.startTrajectory(1, timescale=2.0)
		time.sleep(traj2.duration * 2.0)
	cf.startTrajectory(0, timescale=1.0, reverse=True)
	cf2.startTrajectory(0, timescale=1.0, reverse=True)
	time.sleep(traj1.duration * 1.0)
	cf.stop()
	cf2.stop()

if __name__ == '__main__':
	drones = sys.argv[1:]
	if len(drones != 2):
		print "2Drones_figure8.py [crazyflie1] [crazyflie2]"
	try:
		main()
	except Exception as e:
		print e
