#!/usr/bin/env python
import signal, sys, time
import rospy
import crazyflie
import uav_trajectory
from std_msgs.msg import Float32

cf = None
DRONE_NAME = "crazyflie3"


######################################################
################                   ###################
################   BATTERY LEVEL   ###################
################                   ###################
######################################################
## nedded imports: 
#
# import rospy
# from std_msgs.msg import Float32
#
# set the following values to the max and min battery levels
# to achive the correct battery level in precentge

MAX_BATTERY = 4.04
MIN_BATTERY = 2.64

# callback function that prints the message
def callback(data):
    rospy.loginfo('Battery level is: %i %%', ( ( data.data - MIN_BATTERY ) / (MAX_BATTERY - MIN_BATTERY) ) * 100) # print to screen
# listener to the battery level topic
# for drone number 3 set value to: '/crazyflie3/battery'
def battery_listener():
    rospy.Subscriber('/crazyflie3/battery', Float32, callback)

# set the 'battery_listener()' line were you want the
# battery level to be printed.  

######################################################


#########################################
#############    EXAMPLE    ############# 
#############  for use in   #############
#############  real  code   #############
#########################################

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

	print("starting")
	traj1 = uav_trajectory.Trajectory()
	traj1.loadcsv("takeoff.csv")
	traj2 = uav_trajectory.Trajectory()
	traj2.loadcsv("figure8.csv")
	print(traj1.duration)
	cf.uploadTrajectory(0, 0, traj1)
	cf.uploadTrajectory(1, len(traj1.polynomials), traj2)
	cf.startTrajectory(0, timescale=1.0)
	time.sleep(traj1.duration * 2.0)
	while 1:
		
		battery_listener() # print battey level
		time.sleep(1)
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
