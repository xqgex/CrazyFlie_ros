#!/usr/bin/env python
import signal, sys, time
import rospy
import crazyflie
import uav_trajectory
from std_msgs.msgs import float32

cf = None
DRONE_NAME = "crazyflie2"

def callback(msg):
	x=msg.data

def signal_handler(sig, frame):
	global cf
	try:
		cf.land(targetHeight = 0.0, duration = 1.5)
		cf.stop()
	finally:
		print "Ctrl+C pressed"
		sys.exit(0)

def main():
	signal.signal(signal.SIGINT, signal_handler)
	global cf
	rospy.init_node('test_high_level')
	cf = crazyflie.Crazyflie(DRONE_NAME, DRONE_NAME)
	cf.setParam("commander/enHighLevel", 1)
	rospy.Subscriber("/{}/battery".format(DRONE_NAME),Float32,callback)
	#cf.takeoff(targetHeight = 0.5, duration = 2.0)
	#time.sleep(3.0)
	#cf.goTo(goal = [0.0, 0.0, 0.5], yaw=0.2, duration = 2.0, relative = False)
	#time.sleep(7.0)
	#cf.goTo(goal = [0.5, 0.5, 0.5], yaw=0.2, duration = 2.0, relative = False)
	#time.sleep(7.0)
	#cf.goTo(goal = [0.0, 0.0, 0.5], yaw=0.2, duration = 2.0, relative = False)
	#time.sleep(7.0)
	#cf.land(targetHeight = 0.0, duration = 2.0)
	print("starting")
	traj1 = uav_trajectory.Trajectory()
	traj1.loadcsv("takeoff.csv")
	traj2 = uav_trajectory.Trajectory()
	traj2.loadcsv("figure8.csv") # yuval
	print(traj1.duration)
	cf.uploadTrajectory(0, 0, traj1)
	cf.uploadTrajectory(1, len(traj1.polynomials), traj2)
	cf.startTrajectory(0, timescale=1.0)
	time.sleep(traj1.duration * 2.0)
        # time.sleep(traj1.duration * 1000.0) # yuval
	while 1:
		
		print "battery level is: " + str(bat_level)
		cf.startTrajectory(1, timescale=2.0)
		time.sleep(traj2.duration * 2.0)
	cf.startTrajectory(0, timescale=1.0, reverse=True)
	time.sleep(traj1.duration * 1.0)
	cf.stop()

if __name__ == '__main__':
	try:
		main()
	except Exception as e:
		print e
