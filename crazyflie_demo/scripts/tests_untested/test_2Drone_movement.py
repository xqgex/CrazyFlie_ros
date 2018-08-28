#!/usr/bin/env python
import signal, sys, time
import rospy
import crazyflie
import uav_trajectory

cf1 = None
cf2 = None
DRONE1_NAME = "crazyflie2"
DRONE2_NAME = "crazyflie3"
def signal_handler(sig, frame):
	global cf1
	try:
		cf1.land(targetHeight = 0.0, duration = 1.5)
		cf1.stop()
	finally:
		print "Ctrl+C pressed"
		sys.exit(0)
		
def main():
	SLEEP = 3
	signal.signal(signal.SIGINT, signal_handler)
	global cf1
	global cf2
	rospy.init_node('test_2Drone_movement')
	cf1 = crazyflie.Crazyflie(DRONE1_NAME, DRONE1_NAME)
	cf2 = crazyflie.Crazyflie(DRONE2_NAME, DRONE2_NAME)
	cf1.setParam("commander/enHighLevel", 1)
	cf2.setParam("commander/enHighLevel", 1) #Check later
	print("starting")
	traj1 = uav_trajectory.Trajectory()
	traj1.loadcsv("takeoff.csv")
	time.sleep(traj1.duration * 2.0)
	print("Drone crazyflie2 should take off")
	cf1.uploadTrajectory(0, 0, traj1)
	cf1.startTrajectory(0, timescale=1.0)
	time.sleep(traj1.duration * 2.0)
	time.sleep(SLEEP)
	print("Drone crazyflie3 should take off")
	cf2.uploadTrajectory(0, 0, traj1)
	cf2.startTrajectory(0, timescale=1.0)
	time.sleep(SLEEP)
	#movement after take off
	print("Drones should now go to 1st positions")
	#each drone moves to a different edge
	cf1.goTo(goal = [-0.7, 0, 0.5], yaw=0.2, duration = 2.0, relative = False) 
	cf2.goTo(goal = [1, 0, 0.5], yaw=0.2, duration = 2.0, relative = False)
	time.sleep(SLEEP)
	#Each drone move a bit to the center
	cf1.goTo(goal = [-0.5, 0, 0.5], yaw=0.2, duration = 2.0, relative = False) 
	cf2.goTo(goal = [0.8, 0, 0.5], yaw=0.2, duration = 2.0, relative = False)
	time.sleep(SLEEP)
	#Each drone goes to a diffrent side
	cf1.goTo(goal = [-0.5, 0.4, 0.5], yaw=0.2, duration = 2.0, relative = False) 
	cf2.goTo(goal = [0.8, -0.4, 0.5], yaw=0.2, duration = 2.0, relative = False)
	time.sleep(SLEEP)
	#Each drone goes to a diffrent side
	cf1.goTo(goal = [-0.5, -0.4, 0.5], yaw=0.2, duration = 2.0, relative = False) 
	cf2.goTo(goal = [0.8, 0.4, 0.5], yaw=0.2, duration = 2.0, relative = False)
	time.sleep(SLEEP)
	#Each drone goes a bit to the center
	cf1.goTo(goal = [-0.3, -0.4, 0.5], yaw=0.2, duration = 2.0, relative = False) 
	cf2.goTo(goal = [0.6, 0.4, 0.5], yaw=0.2, duration = 2.0, relative = False)
	time.sleep(SLEEP)
	cf1.land(targetHeight = 0.0, duration = 2.0)
	cf2.land(targetHeight = 0.0, duration = 2.0)
	time.sleep(SLEEP)
	#stop drones	
	cf1.stop()
	cf2.stop()
	
	
	
	
if __name__ == '__main__':
	try:
		main()
	except Exception as e:
		print e
	
