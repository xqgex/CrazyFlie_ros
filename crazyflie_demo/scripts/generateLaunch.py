#!/usr/bin/env python
# -*- coding: utf-8 -*-
import logger, sys
cf_logger = logger.get_logger(__name__) # debug(), info(), warning(), error(), exception(), critical()

def droneNode(current_drone_number):
	return """  <group ns="$(arg frame{num})">
    <node pkg="crazyflie_driver" type="crazyflie_add" name="crazyflie_add" output="screen">
      <param name="uri" value="$(arg uri{num})" />
      <param name="tf_prefix" value="$(arg frame{num})" />
      <param name="enable_logging" value="True" />
      <param name="enable_logging_imu" value="False" />
      <param name="enable_logging_temperature" value="False" />
      <param name="enable_logging_magnetic_field" value="False" />
      <param name="enable_logging_pressure" value="False" />
      <param name="enable_logging_battery" value="True" />
      <param name="enable_logging_packets" value="False" />
      <rosparam>
        genericLogTopics: ["log1"]
        genericLogTopicFrequencies: [100]
        genericLogTopic_log1_Variables: ["stateEstimate.x", "stateEstimate.y", "stateEstimate.z"]
      </rosparam>
    </node>
    <node name="pose" pkg="crazyflie_demo" type="publishPosition.py" args="$(arg frame{num})" output="screen"> 
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
      refresh_tracker_frequency: 0.0
      trackers:
      - $(arg frame{num})
    </rosparam>
    </node>
  </group>""".format(num=current_drone_number)

def ledNode(current_led_name):
	return """  <node pkg="vrpn_client_ros" type="vrpn_client_node" name="vrpn_client_node" output="screen">
    <rosparam subst_value="true">
      server: $(arg ip)
      port: $(arg port)
      update_frequency: 100.0
      frame_id: /world
      child_frame_id: /{led}
      use_server_time: false
      broadcast_tf: true
      refresh_tracker_frequency: 0.0
      trackers:
      - led1
    </rosparam>
  </node>""".format(led=current_led_name)

def fileStracture(ip, port, drones, leds):
	drone_args = []
	for drone_number in range(len(drones)):
		drone_args.append("""  <arg name="uri{num}" default="radio://0/80/2M/E7E7E7E70{mac}" />
  <arg name="frame{num}" default=\"{name}" />""".format(num=drone_number, mac=drones[drone_number][-1], name=drones[drone_number]))
	drone_args = "\n".join(drone_args)
	drones_text = [droneNode(drone_number) for drone_number in range(len(drones))]
	drones_text = "\n".join(drones_text)
	leds_text = [ledNode(led_name) for led_name in leds]
	leds_text = "\n".join(leds_text)
	return """<?xml version="1.0"?>
<launch>
{drone_args}
  <arg name="ip" default="{ip}" />
  <arg name="port" default="{port}" />
  <include file="$(find crazyflie_driver)/launch/crazyflie_server.launch"></include>
{drones_text}
{leds_text}
</launch>
""".format(ip=ip, port=port, drone_args=drone_args, drones_text=drones_text, leds_text=leds_text)

def generate(file_name, ip, port, drones, leds):
	try:
		with open(file_name, "w") as outfile:
			outfile.write(fileStracture(ip, port, drones, leds))
		return True
	except Exception as e:
		cf_logger.exception("Failed to create {}".format(file_name))
		return False

if __name__ == "__main__":
	print "This is not the way to do it..."

