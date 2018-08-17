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
TCP_IP = "172.16.1.2"
TCP_PORT = 51951
BUFFER_SIZE = 1024
WORLD_RANGE = {"X":[-0.65,1.55], "Y":[-0.8,0.45]}
WORLD_CELLS_COUNT = {"X":11, "Y":6}
FLIGHT_HEIGHT = 0.5
#########################

WORLD_STEPS = {"X":-1, "Y":-1} # Will be automaticly generated based on "WORLD_RANGE" and "WORLD_CELLS_COUNT"
WORLD_ORIGIN = {"X":-1, "Y":-1} # Will be automaticly generated
KNOWN_CRAZYFLIES = {}
VALID_DIRECTIONS = ["UP", "DOWN", "RIGHT", "LEFT"]
#		  0	      1		    2	       3       4     5	     6	      7
VALID_COMMANDS = ["Register", "UnRegister", "TakeOff", "Land", "UP", "DOWN", "RIGHT", "LEFT"] # Add new commands only at the end!!

class CrazyFlieObject(object):
	_cf = None
	_listener = None
	_name = None
	_pos = [-1,-1]
	_status = None
	def __init__(self, name):
		self._name = name
		#self._pos = [x,y]
		self._status = "OFF"
		self._listener = tf.TransformListener()
		self._cf = crazyflie.Crazyflie(name, name)
		self._cf.setParam("commander/enHighLevel", 1)
	def checkPosition():
		if self._status != "UP":
			return
		_real_pos,rot = self._listener.lookupTransform("/world", "/{}".format(self._name), rospy.Time(0))
		if _real_pos[2] < 0.1:
			cf_logger.critical("Drone height too low, Aborting...")
			self._status = "OFF"
	def getStatus(self):
		return self._status
	def takeOff(self):
		self._cf.takeoff(targetHeight=FLIGHT_HEIGHT, duration=3)
		time.sleep(3.5)
		self._status = "UP"
	def land(self, landing_duration = 1.5):
		self._cf.land(targetHeight=0.0, duration=landing_duration)
		time.sleep(landing_duration)
		try:
			self._cf.stop()
		except Exception as e:
			cf_logger.exception("Failed to stop CrazyFlie")
		self._status = "OFF"
	def doMovement(self):
		target_pos = cell2pos(self._pos)
		self._cf.goTo(goal=[target_pos[0],target_pos[1],FLIGHT_HEIGHT], yaw=0.0, duration=3, relative=False)
		time.sleep(3.5)
		_real_pos,rot = self._listener.lookupTransform("/world", "/{}".format(self._name), rospy.Time(0))
		cf_logger.info("{} is at ({:.2f},{:.2f},{:.2f}), Should be ({:.2f},{:.2f},{:.2f}), Cell ({},{})".format(self._name,_real_pos[0],_real_pos[1],_real_pos[2],target_pos[0],target_pos[1],FLIGHT_HEIGHT,self._pos[0],self._pos[1]))
	def goTo(self, x, y):
		self._pos = [x,y]
		self.doMovement()
	def move(self, direction):
		if direction == "UP":
			self._pos[1] -= 1 # Decrease the value in Y axis
		elif direction == "DOWN":
			self._pos[1] += 1 # Increase the value in Y axis
		elif direction == "RIGHT":
			self._pos[0] -= 1 # Decrease the value in X axis
		elif direction == "LEFT":
			self._pos[0] += 1 # Increase the value in X axis
		self.doMovement()
	def getPos(self):
		return self._pos

def cell2pos(cell):
	ret_x = round(WORLD_ORIGIN["X"] + (cell[0]*WORLD_STEPS["X"]),2)
	ret_y = round(WORLD_ORIGIN["Y"] + (cell[1]*WORLD_STEPS["Y"]),2)
	return [ret_x,ret_y]

def handleSocket():
	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	s.bind((TCP_IP, TCP_PORT))
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
						if cf_object.getStatus() == "UP":
							cf_logger.warning("Emergency landing for {}".format(cf_name))
							cf_object.land() # WTF
						cf_logger.warning("Auto remove registered drone '{}'".format(cf_name))
						del cf_object
				break
			if 1 <= data.count("$"):
				loop_ret = "INVALID"
				loop_command = data.split("$")
				if loop_command[1] == VALID_COMMANDS[0]: # Register
					if loop_command[0] not in KNOWN_CRAZYFLIES:
						try:
							cf_logger.info("Add new drone: {}".format(loop_command[0]))
							KNOWN_CRAZYFLIES[loop_command[0]] = CrazyFlieObject(loop_command[0]) # Drone is new
							loop_ret = "OK"
						except Exception as e:
							cf_logger.exception("Failed to create CrazyFlie")
				elif loop_command[1] == VALID_COMMANDS[1]: # UnRegister
					if (loop_command[0] in KNOWN_CRAZYFLIES) and (KNOWN_CRAZYFLIES[loop_command[0]].getStatus() == "OFF"):
						cf_logger.info("Remove drone: {}".format(loop_command[0]))
						del KNOWN_CRAZYFLIES[loop_command[0]] # Drone is known and down
						loop_ret = "OK"
				elif loop_command[1] == VALID_COMMANDS[2]: # TakeOff
					if len(loop_command) == 4:
						if (loop_command[0] in KNOWN_CRAZYFLIES) and (KNOWN_CRAZYFLIES[loop_command[0]].getStatus() == "OFF"):
							cf_logger.debug("Takeoff {} and go to ({},{})".format(loop_command[0],loop_command[2],loop_command[3]))
							KNOWN_CRAZYFLIES[loop_command[0]].takeOff() # Drone is known and down
							KNOWN_CRAZYFLIES[loop_command[0]].goTo(int(loop_command[2]),int(loop_command[3]))
							loop_ret = "OK"
				elif loop_command[1] == VALID_COMMANDS[3]: # Land
					if (loop_command[0] in KNOWN_CRAZYFLIES) and (KNOWN_CRAZYFLIES[loop_command[0]].getStatus() == "UP"):
						cf_logger.debug("Land {}".format(loop_command[0]))
						KNOWN_CRAZYFLIES[loop_command[0]].land() # Drone is known and up
						loop_ret = "OK"
				elif loop_command[1] in VALID_DIRECTIONS: # "UP", "DOWN", "RIGHT", "LEFT"
					if (loop_command[0] in KNOWN_CRAZYFLIES) and (KNOWN_CRAZYFLIES[loop_command[0]].getStatus() == "UP"):
						cf_logger.debug("{} moved {}".format(loop_command[0],loop_command[1]))
						KNOWN_CRAZYFLIES[loop_command[0]].move(loop_command[1])
						cur_cell = KNOWN_CRAZYFLIES[loop_command[0]].getPos()
						loop_ret = "OK"
				else:
					cf_logger.error("Received invalid command: {} => {}".format(loop_command[0],loop_command[1]))
				conn.send(loop_ret)
			else:
				cf_logger.error("Invalid data received: '{}'".format(data))
				conn.send("FORMAT")
		cf_logger.info("Closing connection with {}:{}".format(addr[0],addr[1]))
		conn.close()

def main():
	WORLD_STEPS["X"] = round(abs(WORLD_RANGE["X"][1]-WORLD_RANGE["X"][0])/WORLD_CELLS_COUNT["X"],2) # X
	WORLD_STEPS["Y"] = round(abs(WORLD_RANGE["Y"][1]-WORLD_RANGE["Y"][0])/WORLD_CELLS_COUNT["Y"],2) # Y
	WORLD_ORIGIN["X"] = min(WORLD_RANGE["X"][0],WORLD_RANGE["X"][1])
	WORLD_ORIGIN["Y"] = min(WORLD_RANGE["Y"][0],WORLD_RANGE["Y"][1])
	rospy.init_node("the_new_gofetch")
	handleSocket()

if __name__ == "__main__":
	cf_logger.info("######################################################")
	cf_logger.info("####                   Started                    ####")
	cf_logger.info("######################################################")
	main()

