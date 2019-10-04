#!/usr/bin/env python

import time
import rospy
import sys
import signal
import math
import datetime
import tf2_py as tf2
import tf2_ros
import tf2_geometry_msgs
from sensor_msgs import point_cloud2
from tr2py.tr2_sim import TR2
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float64
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

tr2 = TR2()

tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(12))
tf_listener = tf2_ros.TransformListener(tf_buffer)
tf_transform = None

processing_point_cloud = False

x1_bound = -0.45
x2_bound = 0.45
y_bound = 0.20
z_bound = 0.15

def setTransform():
	global tf_transform

	lookup_time = rospy.Time(0)
	target_frame = "base_link"
	source_frame = "camera_link_optical"

	try:
		tf_transform = tf_buffer.lookup_transform(target_frame, source_frame, lookup_time, rospy.Duration(2.0))
	except tf2.LookupException as ex:
		rospy.logwarn(str(lookup_time.to_sec()))
		rospy.logwarn(ex)
	except tf2.ExtrapolationException as ex:
		rospy.logwarn(str(lookup_time.to_sec()))
		rospy.logwarn(ex)

def camera_depth_callback(msg):
	global tf_transform, processing_point_cloud

	if (processing_point_cloud == False):
		processing_point_cloud = True
		time_start = time.clock()
		print "Started..."

		setTransform()

		cloud_out = do_transform_cloud(msg, tf_transform)
		points = list(point_cloud2.read_points(cloud_out, skip_nans=True, field_names=("x", "y", "z")))

		#for p in points:
		#	if (p[0] < x1_bound or p[0] > x2_bound) and p[1] > y_bound and p[2] > z_bound:
		#		print "x: %f, y: %f, z: %f" %(p[0], p[1], p[2])

		time_elapsed = (time.clock() - time_start)
		print "Completed in", time_elapsed, "seconds"
		processing_point_cloud = False

def signal_handler(sig, frame):
	global close
	close = True
	sys.exit(0)
	
signal.signal(signal.SIGINT, signal_handler)

if __name__ == '__main__':
	rospy.Subscriber("/camera/depth/points", PointCloud2, camera_depth_callback)
	tr2.spin()


