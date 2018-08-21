#!/usr/bin/env python
# -*- coding: utf-8 -*-
import GenerateLaunch, logger, os, sys
cf_logger = logger.get_logger(__name__) # debug(), info(), warning(), error(), exception(), critical()

LAUNCH_FOLDER = "../launch"
LAUNCH_FILE = "crazy_game.launch"
LAUNCH_PATH = "{}/{}".format(LAUNCH_FOLDER, LAUNCH_FILE)
IP = "172.16.1.1"
PORT = "3883"

def main(arguments):
	if (len(arguments) == 0) or (4 < len(arguments)):
		cf_logger.error("Invalid call, Usage:")
		cf_logger.error("python main.py {[crazyflie0] ..}")
		cf_logger.error("For example:")
		cf_logger.error("python main.py crazyflie2 crazyflie3")
	elif not GenerateLaunch.generate(LAUNCH_PATH, IP, PORT, arguments):
		cf_logger.info("Failed to create {}".format(LAUNCH_PATH))
 	else:
		cf_logger.info("File {} created successfully".format(LAUNCH_PATH))
		with open("inventory.txt", "w+") as f:
			f.write("\n".join(arguments))
		os.system("roslaunch crazyflie_demo {}".format(LAUNCH_FILE))

if __name__ == "__main__":
	cf_logger.info("######################################################")
	cf_logger.info("####                   Started                    ####")
	cf_logger.info("######################################################")
	main(sys.argv[1:])

