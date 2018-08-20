#!/usr/bin/env python
# -*- coding: utf-8 -*-
import logger, socket, sys, time
cf_logger = logger.get_logger(__name__) # debug(), info(), warning(), error(), exception(), critical()
if sys.version_info[0] > 2:
	cf_logger.warning("Please use Python 2.x")
	print "If you can see this message, Then you are running Python 3 instead of 2"
	exit(0)
try:
	import crazyflie, rospy, tf
except:
	cf_logger.warning("######################################################")
	cf_logger.warning("####              OFFLINE mode is ON              ####")
	cf_logger.warning("######################################################")
	cf_logger.warning("#### Warning, You don't have the crazyflie files  ####")
	cf_logger.warning("#### Loading dummy files for testing purposes     ####")
	cf_logger.warning("######################################################")
	from Simulator import crazyflie, rospy, tf

##### Editable part #####
DEFAULT_LOCAL_IP = "127.0.0.1"
DEFAULT_WIN_IP = "172.16.1.1"
DEFAULT_TCP_PORT = 51951
DEFAULT_BUFFER_SIZE = 1024
WORLD_RANGE = {"X":[-0.65,1.55], "Y":[-0.8,0.45]}
WORLD_CELLS_COUNT = {"X":11, "Y":6}
FLIGHT_HEIGHT = 0.5
#########################

KNOWN_CRAZYFLIES = {}
VALID_COMMANDS = {"TakeOff": "_take_off",	"GetObjects": "_get_objects",	"BatteryStatus": "_battery_status",
		  "GoTo": "_go_to",		"MoveDrone": "_move_drone",	"GetPos": "_get_position",
		  "Land": "_land",		"WorldSize": "_World_size",	"SetSpeed": "_set_speed"}

class CrazyFlieObject(object):
	def __init__(self, name):
		self._name = name
		self._status = "Landed"
		self._move_speed = 1
		self._listener = tf.TransformListener()
		self._cf = crazyflie.Crazyflie(name, self._listener)
		self._cf.setParam("commander/enHighLevel", 1)
	def getPosition(self):
		if self._status == "Landed":
			return
		real_pos = self._cf.position()
		if real_pos[2] < 0.1:
			cf_logger.critical("Drone height too low, Aborting...")
			self._status = "Landed"
		return real_pos
	def getStatus(self):
		return self._status
	def takeOff(self):
		self._cf.takeoff(targetHeight=FLIGHT_HEIGHT, duration=3)
		time.sleep(3.5)
		self._status = "Running"
	def land(self, landing_duration = 1.5):
		self._cf.land(targetHeight=0.0, duration=landing_duration)
		time.sleep(landing_duration)
		try:
			self._cf.stop()
		except Exception as e:
			cf_logger.exception("Failed to stop CrazyFlie")
		self._status = "Landed"
	def goTo(self, x, y):
		self._cf.goTo(goal = [x,y,FLIGHT_HEIGHT], yaw=0.0, duration=1.3, relative=False)
	def relativeMove(self, x, y):
		real_x, real_y, real_z = self._cf.position()
		cf.goTo(goal = [real_x + (x*self._move_speed), real_y + (y*self._move_speed), real_z], yaw=0.0, duration=1.3, relative=False)
	def setSpeed(self, speed):
		self._move_speed = speed

def _get_objects(args): # args = ["GetObjects"]
	# TODO
	tmp_list = ["crazyflie2", "crazyflie3"]# TODO
	# TODO
	for object_name in tmp_list:
		try:
			KNOWN_CRAZYFLIES[object_name] = CrazyFlieObject(object_name)
			cf_logger.info("Drone '{}' added".format(object_name))
		except Exception as e:
			cf_logger.exception("Failed to create CrazyFlie named: {}".format(object_name))
	# TODO return "FATAL" on error
	return "$".join(tmp_list)

def _battery_status(args): # args = ["BatteryStatus", "crazyflie"]
	return "FATAL" # TODO

def _take_off(args): # args = ["TakeOff", "crazyflie"]
	if (len(args) == 2) and (args[1] in KNOWN_CRAZYFLIES) and (KNOWN_CRAZYFLIES[args[1]].getStatus() == "Landed"):
		cf_logger.debug("Takeoff {}".format(args[1]))
		KNOWN_CRAZYFLIES[args[1]].takeOff()
	return

def _land(args): # args = ["Land", "crazyflie"]
	if (len(args) == 2) and (args[1] in KNOWN_CRAZYFLIES) and (KNOWN_CRAZYFLIES[args[1]].getStatus() == "Running"):
		cf_logger.debug("Land {}".format(args[1]))
		KNOWN_CRAZYFLIES[args[1]].land()
	return

def _go_to(args): # args = ["GoTo", "crazyflie", "0', "0"]
	if (len(args) == 4) and (args[1] in KNOWN_CRAZYFLIES) and (KNOWN_CRAZYFLIES[args[1]].getStatus() == "Running"):
		cf_logger.debug("Tell {} go to ({},{})".format(args[1],int(args[2]),int(args[3])))
		KNOWN_CRAZYFLIES[args[1]].goTo(int(args[2]),int(args[3]))
	return

def _move_drone(args): # args = ["MoveDrone", "crazyflie", "0.992350", "0.123456"]
	if (len(args) == 4) and (args[1] in KNOWN_CRAZYFLIES) and (KNOWN_CRAZYFLIES[args[1]].getStatus() == "Running"):
		KNOWN_CRAZYFLIES[args[1]].relativeMove(int(args[2]),int(args[3]))
	return

def _get_position(args): # args = ["GetPos", "crazyflie"]
	if (len(args) == 2) and (args[1] in KNOWN_CRAZYFLIES) and (KNOWN_CRAZYFLIES[args[1]].getStatus() == "Running"):
		pos = KNOWN_CRAZYFLIES[args[1]].getPosition()
		cf_logger.debug("The position of {} is {}".format(args[1], pos))
		return "$".join([str(a) for a in pos])
	else:
		return

def _set_speed(args): # args = ["SetSpeed", "6"]
	if len(args) == 2:
		for cf_name, cf_object in KNOWN_CRAZYFLIES.iteritems():
			cf_object.setSpeed(int(args[1]))
	return

def _World_size(args): # args = ["WorldSize"]
	size_x = WORLD_RANGE["X"][1] - WORLD_RANGE["X"][0]
	size_y = WORLD_RANGE["Y"][1] - WORLD_RANGE["Y"][0]
	return "{}${}".format(size_x, size_y)

def handleSocket(ip=DEFAULT_LOCAL_IP):
	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
	s.bind((ip, DEFAULT_TCP_PORT))	
	s.listen(1) # Accept only one connection at a time
	while True: # Keep waiting for connection
		cf_logger.info("Waiting for incoming connection")
		try:
			conn, addr = s.accept()
		except KeyboardInterrupt:
			cf_logger.warning("Keyboard Interrupt")
			break
		except Exception as e:
			cf_logger.exception("Exception occurred")
			break
		cf_logger.info("######################################################")
		cf_logger.info("New connection from: {}:{}".format(addr[0],addr[1]))
		while 1:
			data = conn.recv(DEFAULT_BUFFER_SIZE)
			if not data:
				if KNOWN_CRAZYFLIES: # There are still registered drones
					for cf_name, cf_object in KNOWN_CRAZYFLIES.iteritems():
						if cf_object.getStatus() == "Running":
							cf_logger.warning("Emergency landing for {}".format(cf_name))
							cf_object.land()
						cf_logger.warning("Auto remove registered drone '{}'".format(cf_name))
						del cf_name
				break
			data = data.split("$")
			if data[0] in VALID_COMMANDS:
				result = globals()[VALID_COMMANDS[data[0]]](data)
				if result:
					cf_logger.info("Send back '{}'".format(result))
					conn.send(result)
				else:
					conn.send("\0")
			else:
				cf_logger.error("Invalid data received: '{}'".format(data))
				conn.send("FATAL")
		cf_logger.info("Closing connection with {}:{}".format(addr[0],addr[1]))
		conn.close()

def main():
	rospy.init_node("the_new_gofetch")
	handleSocket()#ip=DEFAULT_WIN_IP) # TODO

if __name__ == "__main__":
	cf_logger.info("######################################################")
	cf_logger.info("####                   Started                    ####")
	cf_logger.info("######################################################")
	main()

