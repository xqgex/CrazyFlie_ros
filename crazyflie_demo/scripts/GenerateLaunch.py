#!/usr/bin/env python
# -*- coding: utf-8 -*-
import logger, sys
cf_logger = logger.get_logger(__name__) # debug(), info(), warning(), error(), exception(), critical()

FILE_NAME = "../launch/crazy_game.launch"
IP = "172.16.1.1"
PORT = "3883"

def droneNode(current_drone_number, drones_count):
	if current_drone_number+1 == drones_count:
		trackers = "      trackers:\n"
		for n in range(drones_count):
			trackers += "      - $(arg frame{})\n".format(n)
	else:
		trackers = ""
	return """  <group ns="$(arg frame{num})">
    <node pkg="crazyflie_driver" type="crazyflie_add" name="crazyflie_add" output="screen">
      <param name="uri" value="$(arg uri{num})" />
      <param name="tf_prefix" value="$(arg frame{num})" />
      <param name="enable_logging" value="False" />
      <param name="enable_logging_imu" value="False" />
      <param name="enable_logging_temperature" value="False" />
      <param name="enable_logging_magnetic_field" value="False" />
      <param name="enable_logging_pressure" value="False" />
      <param name="enable_logging_battery" value="False" />
      <param name="enable_logging_packets" value="False" />
      <rosparam>
        genericLogTopics: ["log1"]
        genericLogTopicFrequencies: [100]
        genericLogTopic_log1_Variables: ["stateEstimate.x", "stateEstimate.y", "stateEstimate.z"]
      </rosparam>
    </node>
    <node name="pose" pkg="crazyflie_demo" type="crazygame_publish_position.py" args="$(arg frame{num})" output="screen"> 
      <param name="topic" value="/$(arg frame{num})/vrpn_client_node/$(arg frame{num})/pose" />
    </node>
    <node pkg="vrpn_client_ros" type="vrpn_client_node" name="vrpn_client_node" output="screen">
    <rosparam subst_value="true">
      server: $(arg ip)
      port: $(arg port)
      update_frequency: 100.0
      frame_id: /world
      child_frame_id: $(arg frame{num})
      use_server_time: false
      broadcast_tf: true
      refresh_tracker_frequency: 1.0
{trackers}    </rosparam>
    </node>
  </group>""".format(num=current_drone_number, trackers=trackers)

def fileStracture(arguments):
	drone_args = []
	for drone_number in range(len(arguments)):
		drone_args.append("""  <arg name="uri{num}" default="radio://0/80/2M/E7E7E7E70{mac}" />
  <arg name="frame{num}" default=\"{name}" />""".format(num=drone_number, mac=arguments[drone_number][-1], name=arguments[drone_number]))
	drone_args = "\n".join(drone_args)
	drones_text = [droneNode(drone_number,len(arguments)) for drone_number in range(len(arguments))]
	drones_text = "\n".join(drones_text)
	return """<?xml version="1.0"?>
<launch>
{drone_args}
  <arg name="ip" default="{ip}" />
  <arg name="port" default="{port}" />
  <include file="$(find crazyflie_driver)/launch/crazyflie_server.launch"></include>
{drones_text}
</launch>
""".format(ip=IP, port=PORT, drone_args=drone_args, drones_text=drones_text)

def main(arguments):
	if (len(arguments) == 0) or (4 < len(arguments)):
		cf_logger.error("Invalid call, Usage:")
		cf_logger.error("python GenerateLaunch.py {[crazyflie0] ..}")
		cf_logger.error("For example:")
		cf_logger.error("python GenerateLaunch.py crazyflie2 crazyflie3")
	else:
		with open(FILE_NAME, "w") as outfile:
			outfile.write(fileStracture(arguments))
		cf_logger.info("File {} created successfully".format(FILE_NAME))

if __name__ == "__main__":
	cf_logger.info("######################################################")
	cf_logger.info("####                   Started                    ####")
	cf_logger.info("######################################################")
	main(sys.argv[1:])

