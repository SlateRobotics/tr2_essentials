#!/usr/bin/env python

import time
import rospy
import sys
import signal
import math
import datetime
import ros_numpy
import numpy as np
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

def quaternion_multiply(quaternion1, quaternion0):
    w0, x0, y0, z0 = quaternion0
    w1, x1, y1, z1 = quaternion1
    return np.array([-x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
                     x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
                     -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
                     x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0], dtype=np.float64)

def scale(a, shape):
	out = np.empty(shape).astype(np.ndarray)
	for row in range(shape[0]):
		for col in range(shape[1]):
			r1 = row * (a.shape[0] / shape[0])
			c1 = col * (a.shape[1] / shape[1])
			out[row][col] = a[r1][c1]
	return out

def camera_depth_callback(msg):
	global tf_transform, processing_point_cloud

	if (processing_point_cloud == False):
		processing_point_cloud = True
		time_start = time.clock()
		print "Started..."

		setTransform()

		cloud_in = ros_numpy.numpify(msg).astype(np.ndarray)
		cloud_in = scale(cloud_in, (48, 64))
		cloud_out = np.empty((cloud_in.shape[0], cloud_in.shape[1])).astype(np.ndarray)

		tran = tf_transform.transform.translation
		rot = tf_transform.transform.rotation

		r1 = [rot.w, rot.x, rot.y, rot.z]
		r2 = [rot.w, -rot.x, -rot.y, -rot.z]
		t1 = [tran.x, tran.y, tran.z]
		
		for row in range(cloud_in.shape[0]):
			for col in range(cloud_in.shape[1]):
				p =  cloud_in[row][col]
				p1 = [0, p[0], p[1], p[2]]

				p2 = quaternion_multiply(quaternion_multiply(r1, p1), r2)
				p2 = [p2[1], p2[2], p2[3]]
				p2 = np.add(p2, t1)
				cloud_out[row][col] = [p2[0], p2[1], p2[2], 0]

		print cloud_in[45][0]
		print cloud_out[45][0]

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

	'''cloud_out = do_transform_cloud(msg, tf_transform)
	points = list(point_cloud2.read_points(cloud_out, skip_nans=False, field_names=("x", "y", "z")))
	points2 = list(point_cloud2.read_points(msg, skip_nans=False, field_names=("x", "y", "z")))
	print len(points)
	print cloud_in.shape
	print points2[450*640+1]
	print points[450*640+1]

	#for p in points:
	#	if (p[0] < x1_bound or p[0] > x2_bound) and p[1] > y_bound and p[2] > z_bound:
	#		print "x: %f, y: %f, z: %f" %(p[0], p[1], p[2])'''


