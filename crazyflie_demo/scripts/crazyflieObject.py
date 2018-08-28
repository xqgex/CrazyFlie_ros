#!/usr/bin/env python
# -*- coding: utf-8 -*-
import crazyflie, logger, tf, time
cf_logger = logger.get_logger(__name__) # debug(), info(), warning(), error(), exception(), critical()

class CrazyflieObject(object):
	def __init__(self, name, world_size_x, world_size_y, flight_height): # world_size_x = WORLD_RANGE["X"][0] # world_size_y = WORLD_RANGE["Y"][0]
		self._name = name
		self._status = "Landed"
		self._move_speed = 0.2
		self._step_size = 0.2
		self._world_size_x = world_size_x
		self._world_size_y = world_size_y
		self._flight_height = flight_height
		self._listener = tf.TransformListener()
		self._cf = crazyflie.Crazyflie(name, self._listener)
		self._cf.setParam("commander/enHighLevel", 1)
		self._cf.monitorBattery()
	def getPosition(self):
		real_pos = self._cf.position()
		return [real_pos[0] - self._world_size_x, real_pos[1] - self._world_size_y, real_pos[2]]
	def getStatus(self):
		return self._status
	def getSpeed(self):
		return self._move_speed
	def calcDuration(self, distance):
		return distance/self._move_speed
	def getBattery(self):
		return self._cf.getBattery()
	def takeOff(self, takeoff_duration = 3):
		self._cf.takeoff(targetHeight=self._flight_height, duration=takeoff_duration)
		self._status = "Running"
	def land(self, landing_duration = 1.5):
		self._cf.land(targetHeight=0.0, duration=landing_duration)
		time.sleep(landing_duration + 0.5)
		try:
			self._cf.stop()
		except Exception as e:
			cf_logger.exception("Failed to stop Crazyflie")
		self._status = "Landed"
	def goTo(self, x, y, duration=1):
		self._cf.goTo(goal = [x + self._world_size_x, y + self._world_size_y, self._flight_height], yaw=0.0, duration=duration, relative=False)
	def relativeMove(self, x, y):
		real_x, real_y, real_z = self._cf.position()
		cf_logger.debug("From ({rx}, {ry}, {rz})\nself._cf.goTo(goal = [{x}, {y}, {z}], yaw=0.0, duration={d}, relative=False)".format(x=real_x + (x*self._step_size), y=real_y + (y*self._step_size), z=self._flight_height, d=self._step_size/self._move_speed, rx=real_x, ry=real_y, rz=real_z)) # XXX
		self._cf.goTo(goal = [real_x + (x*self._step_size), real_y + (y*self._step_size), self._flight_height], yaw=0.0, duration=self._step_size/self._move_speed, relative=False)
	def setSpeed(self, speed):
		self._move_speed = speed
	def setStepSize(self, step_size):
		self._step_size = step_size

if __name__ == "__main__":
	print "This is not the way to do it..."

