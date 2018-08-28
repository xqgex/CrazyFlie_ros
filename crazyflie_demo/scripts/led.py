#!/usr/bin/env python

import rospy, tf

class Led:
    def __init__(self, name):
        self.tf = tf.TransformListener()
        self.prefix = name

    def getPosition(self):
        self.tf.waitForTransform("/world", "/{}".format(self.prefix), rospy.Time(0), rospy.Duration(10))
        position, quaternion = self.tf.lookupTransform("/world", "/{}".format(self.prefix), rospy.Time(0))
        return np.array(position)
