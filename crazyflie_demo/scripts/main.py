#!/usr/bin/env python
# -*- coding: utf-8 -*-
import generateLaunch, logger, os, sys
cf_logger = logger.get_logger(__name__) # debug(), info(), warning(), error(), exception(), critical()

LAUNCH_FOLDER = "../launch"
LAUNCH_FILE = "crazy_game.launch"
LAUNCH_PATH = "{}/{}".format(LAUNCH_FOLDER, LAUNCH_FILE)
INVENTORY_FILE = "inventory.txt"
IP = "172.16.1.1"
PORT = "3883"

def renameArguments(arguments):
	good_drones = []
	good_leds = []
	for argument in arguments:
		if (len(argument) == 10) and (argument[:9] == "crazyflie") and (48 <= ord(argument[-1]) <= 57):
			good_drones.append(argument)
		elif (len(argument) == 1) and (48 <= ord(argument[-1]) <= 57):
			good_drones.append("crazyflie{}".format(argument))
		elif (len(argument) == 4) and (argument[:3] == "led") and (48 <= ord(argument[-1]) <= 57):
			good_leds.append(argument)
	return [good_drones, good_leds]

def main(arguments):
	drones, leds = renameArguments(arguments)
	if (len(drones) == 0) or (4 < len(drones)):
		cf_logger.error("Invalid call, Usage:")
		cf_logger.error("python main.py {[crazyflie0] ..}")
		cf_logger.error("For example:")
		cf_logger.error("python main.py crazyflie2 crazyflie3")
		cf_logger.error("Another equivalent usage example is:")
		cf_logger.error("python main.py 2 3")
	elif not generateLaunch.generate(LAUNCH_PATH, IP, PORT, drones, leds):
		cf_logger.info("Failed to create {}".format(LAUNCH_PATH))
 	else:
		cf_logger.info("File {} created successfully".format(LAUNCH_PATH))
		with open(INVENTORY_FILE, "w+") as f:
			f.write("\n".join(drones + leds))
		os.system("roslaunch crazyflie_demo {}".format(LAUNCH_FILE))

if __name__ == "__main__":
	cf_logger.info("######################################################")
	cf_logger.info("####                   Started                    ####")
	cf_logger.info("######################################################")
	main(sys.argv[1:])

