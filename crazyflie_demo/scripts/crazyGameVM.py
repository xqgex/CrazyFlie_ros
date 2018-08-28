#!/usr/bin/env python
# -*- coding: utf-8 -*-
import logger, math, socket, sys, time
from crazyflieObject import CrazyflieObject
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
	from simulator import crazyflie, rospy, tf

##### Editable part #####
DEFAULT_LOCAL_IP = "127.0.0.1"
DEFAULT_VM_IP = "172.16.1.2"
DEFAULT_TCP_PORT = 51951
DEFAULT_BUFFER_SIZE = 1024
INVENTORY_FILE = "inventory.txt"
WORLD_RANGE = {"X":[-1.04,1.42], "Y":[-0.9,0.82]} # {"X":[-1.14,1.52], "Y":[-1.0,0.92]}
FLIGHT_HEIGHT = 0.5
#########################

KNOWN_CRAZYFLIES = {}
KNOWN_LEDS = []
VALID_COMMANDS = {
		  "BatteryStatus":	"_battery_status",
		  "GetDrones":		"_get_drones",
		  "GetLeds":		"_get_leds",
		  "GetPos":		"_get_position",
		  "GoTo":		"_go_to",
		  "Land":		"_land",
		  "MoveDrone":		"_move_drone",
		  "SetSpeed":		"_set_speed",
		  "SetStepSize":	"_set_step_size"
		  "TakeOff":		"_take_off",
		  "TakeYourPlace":	"_take_place",
		  "WorldSize":		"_world_size",
		}

def _check_object_type(name):
	if (len(argument) == 10) and (argument[:9] == "crazyflie") and (48 <= ord(argument[-1]) <= 57):
		return "drone"
	elif (len(argument) == 4) and (argument[:3] == "led") and (48 <= ord(argument[-1]) <= 57):
		return "led"
	else:
		return ""

def _battery_status(args): # args = ["BatteryStatus", "crazyflie"]
	if (len(args) == 2) and (args[1] in KNOWN_CRAZYFLIES):
		bat = KNOWN_CRAZYFLIES[args[1]].getBattery()
		return str(bat)
	else:
		return "FATAL"

def _get_drones(args): # args = ["GetDrones"]
	with open(INVENTORY_FILE, "rb") as f:
		objects = [x.strip() for x in f.readlines()]
	drones_name = []
	for object_name in objects:
		if _check_object_type(object_name) == "drone":
			try:
				KNOWN_CRAZYFLIES[object_name] = CrazyflieObject(object_name, WORLD_RANGE["X"][0], WORLD_RANGE["Y"][0])
				cf_logger.info("Drone '{}' added".format(object_name))
				drones_name.append(object_name)
			except Exception as e:
				cf_logger.exception("Failed to create Crazyflie named: {}".format(object_name))
				return "FATAL"
	return "$".join(drones_name)

def _get_leds(args): # args = ["GetLeds"]
	if len(KNOWN_LEDS) == 0: # You cannot add more leds during the game
		with open(INVENTORY_FILE, "rb") as f:
			objects = [x.strip() for x in f.readlines()]
		for object_name in objects:
			if _check_object_type(object_name) == "led":
				KNOWN_LEDS.append(object_name)
	return "$".join(KNOWN_LEDS)

def _get_position(args): # args = ["GetPos", "crazyflie"]
	if len(args) == 2:
		if args[1] in KNOWN_CRAZYFLIES):
			pos = KNOWN_CRAZYFLIES[args[1]].getPosition()
			return "$".join([str(a) for a in pos])
		elif args[1] in KNOWN_LEDS:
			return "0$0$0" # TODO
	return

def _go_to(args): # args = ["GoTo", "crazyflie", "0', "0"]
	if (len(args) == 4) and (args[1] in KNOWN_CRAZYFLIES) and (KNOWN_CRAZYFLIES[args[1]].getStatus() == "Running"):
		pos = KNOWN_CRAZYFLIES[args[1]].getPosition()
		distance = math.hypot(float(args[2]) - pos[0], float(args[3]) - pos[1])
		duration = KNOWN_CRAZYFLIES[args[1]].calcDuration(distance)
		cf_logger.debug("Tell {} go from {} to ({},{}) duration is {}".format(args[1], pos, float(args[2]), float(args[3]), duration )) 
		try:
			KNOWN_CRAZYFLIES[args[1]].goTo(float(args[2]), float(args[3]), duration)
		except Exception as e:
			cf_logger.exception("Exception occurred")
	return

def _land(args): # args = ["Land", "crazyflie"]
	if (len(args) == 2) and (args[1] in KNOWN_CRAZYFLIES) and (KNOWN_CRAZYFLIES[args[1]].getStatus() == "Running"):
		cf_logger.debug("Land {}".format(args[1]))
		KNOWN_CRAZYFLIES[args[1]].land()
	return

def _move_drone(args): # args = ["MoveDrone", "crazyflie", "0.992350", "0.123456"]
	if (len(args) == 4) and (args[1] in KNOWN_CRAZYFLIES) and (KNOWN_CRAZYFLIES[args[1]].getStatus() == "Running"):
		cf_logger.debug("Move {} with vector ({}, {})".format(args[1], float(args[2]), float(args[3])))
		try:
			KNOWN_CRAZYFLIES[args[1]].relativeMove(float(args[2]), float(args[3]))
		except Exception as e:
			cf_logger.exception("Exception occurred")
	return

def _set_speed(args): # args = ["SetSpeed", "6"]
	if len(args) == 2:
		cf_logger.debug("Set speed to be {}".format(args[1]))
		for cf_name, cf_object in KNOWN_CRAZYFLIES.iteritems():
			cf_object.setSpeed(float(args[1]))
	return

def _set_step_size(args): # args = ["SetStepSize", "2"]
	if len(args) == 2:
		cf_logger.debug("Set step size to be {}".format(args[1]))
		for cf_name, cf_object in KNOWN_CRAZYFLIES.iteritems():
			cf_object.setStepSize(float(args[1]))
	return

def _take_off(args): # args = ["TakeOff", "crazyflie"]
	if (len(args) == 2) and (args[1] in KNOWN_CRAZYFLIES) and (KNOWN_CRAZYFLIES[args[1]].getStatus() == "Landed"):
		cf_logger.debug("Takeoff {}".format(args[1]))
		KNOWN_CRAZYFLIES[args[1]].takeOff()
	return

def _take_place(args): # args = ["TakeYourPlace", "crazyflie", "0', "0"]
	if (len(args) == 4) and (args[1] in KNOWN_CRAZYFLIES) and (KNOWN_CRAZYFLIES[args[1]].getStatus() == "Running"):
		requested_x = float(args[2])
		requested_y = float(args[3])
		cf_logger.debug("Tell {} start game from ({},{})".format(args[1], requested_x, requested_y))
		try:
			pos = KNOWN_CRAZYFLIES[args[1]].getPosition()
			duration = math.hypot(requested_x - pos[0], requested_y - pos[1])
			duration /= KNOWN_CRAZYFLIES[args[1]].getSpeed()
			cf_logger.debug("Starting at ({},{}) with duration of {}".format(pos[0], pos[1], duration))
			KNOWN_CRAZYFLIES[args[1]].goTo(requested_x, requested_y, duration=duration)
			time.sleep(duration)
		except Exception as e:
			cf_logger.exception("Exception occurred")
	return

def _world_size(args): # args = ["WorldSize"]
	size_x = WORLD_RANGE["X"][1] - WORLD_RANGE["X"][0]
	size_y = WORLD_RANGE["Y"][1] - WORLD_RANGE["Y"][0]
	return "{}${}".format(size_x, size_y)

def main(ip=DEFAULT_VM_IP):
	rospy.init_node("the_new_gofetch")
	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
	cf_logger.info("Start new server at {}:{}".format(ip, DEFAULT_TCP_PORT))
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
			if e.strerror == "Interrupted system call":
				cf_logger.warning("Keyboard Interrupt")
				break
			else:
				continue
		cf_logger.info("######################################################")
		cf_logger.info("New connection from: {}:{}".format(addr[0],addr[1]))
		while 1:
			try:
				data = conn.recv(DEFAULT_BUFFER_SIZE)
			except KeyboardInterrupt:
				cf_logger.warning("Keyboard Interrupt")
				break
			except Exception as e:
				cf_logger.exception("Exception occurred")
				if e.strerror == "Interrupted system call":
					cf_logger.warning("Keyboard Interrupt")
					break
				else:
					continue
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
					if data[0] != "GetPos":
						cf_logger.info("Send back '{}'".format(result))
					conn.send(result)
				else:
					conn.send("\0")
			else:
				cf_logger.error("Invalid data received: '{}'".format(data))
				conn.send("FATAL")
		cf_logger.info("Closing connection with {}:{}".format(addr[0],addr[1]))
		conn.close()

if __name__ == "__main__":
	cf_logger.info("######################################################")
	cf_logger.info("####                   Started                    ####")
	cf_logger.info("######################################################")
	main()#ip=DEFAULT_LOCAL_IP)

