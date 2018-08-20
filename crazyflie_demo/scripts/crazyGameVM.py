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
DEFAULT_WIN_IP = "172.16.1.2"
TCP_PORT = 51951
BUFFER_SIZE = 1024
WORLD_RANGE = {"X":[-0.65,1.55], "Y":[-0.8,0.45]}
WORLD_CELLS_COUNT = {"X":11, "Y":6}
FLIGHT_HEIGHT = 0.5
#########################

WORLD_STEPS = {"X":-1, "Y":-1} # Will be automaticly generated based on "WORLD_RANGE" and "WORLD_CELLS_COUNT"
WORLD_ORIGIN = {"X":-1, "Y":-1} # Will be automaticly generated
KNOWN_CRAZYFLIES = {}

class CrazyFlieObject(object):
	def __init__(self, name):
		self._name = name
		#self._pos = [x,y]
		self._status = "Landed"
		self._listener = tf.TransformListener()
		self._cf = crazyflie.Crazyflie(name, name)
		self._cf.setParam("commander/enHighLevel", 1)
	def getPosition(self):
		if self._status == "Landed":
			return
		_real_pos,rot = self._listener.lookupTransform("/world", "/{}".format(self._name), rospy.Time(0))
		if _real_pos[2] < 0.1:
			cf_logger.critical("Drone height too low, Aborting...")
			self._status = "Landed"
		return _real_pos
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
	def doMovement(self):
		target_pos = cell2pos(self._pos)
		self._cf.goTo(goal=[target_pos[0],target_pos[1],FLIGHT_HEIGHT], yaw=0.0, duration=3, relative=False)
		time.sleep(3.5)
		_real_pos,rot = self._listener.lookupTransform("/world", "/{}".format(self._name), rospy.Time(0))
		cf_logger.info("{} is at ({:.2f},{:.2f},{:.2f}), Should be ({:.2f},{:.2f},{:.2f}), Cell ({},{})".format(self._name,_real_pos[0],_real_pos[1],_real_pos[2],target_pos[0],target_pos[1],FLIGHT_HEIGHT,self._pos[0],self._pos[1]))
	def goTo(self, x, y):
		self._pos = [x,y]
		self.doMovement()
	def getPos(self):
		return self._pos

def cell2pos(cell):
	ret_x = round(WORLD_ORIGIN["X"] + (cell[0]*WORLD_STEPS["X"]),2)
	ret_y = round(WORLD_ORIGIN["Y"] + (cell[1]*WORLD_STEPS["Y"]),2)
	return [ret_x,ret_y]

#		  0	      1		    2	       3       4     5	     6	      7
VALID_COMMANDS = ["Register", "UnRegister", "TakeOff", "Land", "UP", "DOWN", "RIGHT", "LEFT"] # Add new commands only at the end!!
VALID_COMMANDS = {"TakeOff": "_take_off", "GetObjects": "_get_objects", "Land": "_land", "GoTo": "_go_to", "MoveDrone": "_move_drone", "GetPos": "_get_position", "BatteryStatus": "_battery_status", "WorldSize": "_World_size", "SetSpeed": "_set_speed"}

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
	pass # TODO

def _get_position(args): # args = ["GetPos", "crazyflie"]
	if (len(args) == 2) and (args[1] in KNOWN_CRAZYFLIES) and (KNOWN_CRAZYFLIES[args[1]].getStatus() == "Running"):
		pos = KNOWN_CRAZYFLIES[args[1]].getPosition()
		cf_logger.debug("The position of {} is {}".format(args[1], pos))
		return "$".join([str(a) for a in pos])
	else:
		return

def _battery_status(args): # args = ["BatteryStatus", "crazyflie"]
	return "FATAL" # TODO

def _set_speed(args): # args = ["SetSpeed"]
	return "FATAL" # TODO

def _World_size(args): # args = ["WorldSize"]
	return "{}${}".format(WORLD_CELLS_COUNT["X"], WORLD_CELLS_COUNT["Y"])

def handleSocket(ip=DEFAULT_LOCAL_IP):
	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
	s.bind((ip, TCP_PORT))	
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
			data = conn.recv(BUFFER_SIZE)
			if not data:
				if KNOWN_CRAZYFLIES: # There are still registered drones
					for cf_name,cf_object in KNOWN_CRAZYFLIES.iteritems():
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
	WORLD_STEPS["X"] = round(abs(WORLD_RANGE["X"][1]-WORLD_RANGE["X"][0])/WORLD_CELLS_COUNT["X"],2) # X
	WORLD_STEPS["Y"] = round(abs(WORLD_RANGE["Y"][1]-WORLD_RANGE["Y"][0])/WORLD_CELLS_COUNT["Y"],2) # Y
	WORLD_ORIGIN["X"] = min(WORLD_RANGE["X"][0],WORLD_RANGE["X"][1])
	WORLD_ORIGIN["Y"] = min(WORLD_RANGE["Y"][0],WORLD_RANGE["Y"][1])
	rospy.init_node("the_new_gofetch")
	handleSocket()#ip=DEFAULT_WIN_IP) # TODO

if __name__ == "__main__":
	cf_logger.info("######################################################")
	cf_logger.info("####                   Started                    ####")
	cf_logger.info("######################################################")
	main()

