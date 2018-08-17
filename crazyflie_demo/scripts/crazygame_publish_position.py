#!/usr/bin/env python
import rospy, tf, sys
from geometry_msgs.msg import PointStamped, TransformStamped, PoseStamped #PoseStamped added to support vrpn_client
from crazyflie_driver.srv import UpdateParams

def onNewTransform(pose):
	global msg
	global pub
	global firstTransform
	if firstTransform:
		# initialize kalman filter
		rospy.set_param("kalman/initialX", pose.pose.position.x)
		rospy.set_param("kalman/initialY", pose.pose.position.y)
		rospy.set_param("kalman/initialZ", pose.pose.position.z)
		update_params(["kalman/initialX", "kalman/initialY", "kalman/initialZ"])
		rospy.set_param("kalman/resetEstimation", 1)
		update_params(["kalman/resetEstimation"]) 
		firstTransform = False
	else:
		msg.header.frame_id = pose.header.frame_id
		msg.header.stamp = pose.header.stamp
		msg.header.seq += 1
		msg.point.x = pose.pose.position.x
		msg.point.y = pose.pose.position.y
		msg.point.z = pose.pose.position.z
		pub.publish(msg)

if __name__ == "__main__":
	drones = sys.argv[1:-2]
	drones = ["crazyflie2"] # XXX
	rospy.init_node("publish_external_position_vrpn", anonymous=True)
	rospy.loginfo("start with {}".format(drones))
	topics = []
	for drone in drones:
		topics.append(rospy.get_param("~topic", "/{0}/vrpn_client_node/{0}/pose".format(drone)))
	rospy.loginfo("B4 wait")
	rospy.wait_for_service("update_params")
	rospy.loginfo("found update_params service")
	update_params = rospy.ServiceProxy("update_params", UpdateParams)
	firstTransform = True
	msg = PointStamped()
	msg.header.seq = 0
	msg.header.stamp = rospy.Time.now()
	pub = rospy.Publisher("external_position", PointStamped, queue_size=1)
	for topic in topics:
		rospy.Subscriber(topic, PoseStamped, onNewTransform)
		rospy.spin()

