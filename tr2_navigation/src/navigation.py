#!/usr/bin/env python

import time
import rospy
import sys
import signal
from sensor_msgs.msg import PointCloud2
from tr2py.tr2_sim import TR2
from tr2_navigation import TR2_Nav

tr2 = TR2()
tr2_nav = None

processing = False

def signal_handler(sig, frame):
	sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

def camera_depth_callback(msg):
	global processing

	if (processing == False):
		processing = True
		time_start = time.clock()

		(b0, b1) = tr2_nav.step(msg)
		tr2.b0.setVelocity(b0)
		tr2.b1.setVelocity(b1)

		tr2.h0.setPosition(0.0)
		tr2.h1.setPosition(-0.2)

		rospy.loginfo(str(b0) + ", " + str(b1))

		time_elapsed = (time.clock() - time_start)
		rospy.loginfo("PointCloud processed in " + str(time_elapsed) + " sec")
		processing = False

if __name__ == '__main__':
	rospy.Subscriber("/camera/depth/points", PointCloud2, camera_depth_callback)
	tr2_nav = TR2_Nav()
	tr2.spin()
