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

tr2_b0_pub = None
tr2_b1_pub = None

tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(12))
tf_listener = tf2_ros.TransformListener(tf_buffer)
tf_transform = None

processing_point_cloud = False
avoid_vectors = []

av_coef = 0.01

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

def getPoint(a, i, j, b):
	if (i < 0 or i >= 480):
		return None
	elif (j < 0 or j >= 640):
		return None
	else:
		return a[i][j][b]

def mean_axis(a, i, j, b):
	mean = 0
	size = 0
	points = [getPoint(a, i-1, j-1, b), getPoint(a, i, j-1, b), getPoint(a, i+1, j-1, b), getPoint(a, i-1, j, b), getPoint(a, i, j, b), getPoint(a, i+1, j, b), getPoint(a, i-1, j+1, b), getPoint(a, i, j+1, b), getPoint(a, i+1, j+1, b)]
	for p in points:
		if p != None and math.isnan(p) == False:
			mean = mean + p
			size = size + 1
	if size == 0:
		return None
	else:
		return mean / size

# can do something like this to average the data
# and it shouldn't slow things down too much
def mean(a, i, j):
	return [mean_axis(a, i, j, 0), mean_axis(a, i, j, 1), mean_axis(a, i, j, 2), a[i][j][3]]

def scale(a, shape):
	out = np.empty(shape).astype(np.ndarray)
	for row in range(shape[0]):
		for col in range(shape[1]):
			r1 = row * (a.shape[0] / shape[0])
			c1 = col * (a.shape[1] / shape[1])
			#a[r1][c1] = mean(a, r1, c1)
			out[row][col] = a[r1][c1]
	return out
	
def should_avoid(x, y, z, dist):
	dist_max = 3.0
	z_min = 0.040

	tr2_x1 = -0.35
	tr2_x2 = 0.35
	tr2_y1 = 0.40
	return ((z > z_min) and (dist < dist_max) and ((x < tr2_x1 or x > tr2_x2) and y > tr2_y1))
	
def compute_trajectory(x, y):
	b0_vel = 0
	b1_vel = 0
	
	max_norm = abs(x)
	if max_norm < abs(y):
		max_norm = abs(y)
		
	x_norm = x / max_norm
	y_norm = y / max_norm
	
	max_vel = 4.0
	wheel_radius = 0.200
	wheel_width = 0.6038
	
	omega = -x_norm

	b0_vel = (2*max_vel + omega*wheel_width)/(2*wheel_radius);
	b1_vel = (2*max_vel - omega*wheel_width)/(2*wheel_radius);
	
	if b0_vel > max_vel or b1_vel > max_vel:
		b0_vel = b0_vel / (abs(b0_vel) + abs(b1_vel)) * max_vel
		b1_vel = b1_vel / (abs(b0_vel) + abs(b1_vel)) * max_vel
		
	print b0_vel, b1_vel

	tr2_b0_pub.publish(b0_vel)
	tr2_b1_pub.publish(b1_vel)

def camera_depth_callback(msg):
	global tf_transform, processing_point_cloud, avoid_vectors

	if (processing_point_cloud == False):
		processing_point_cloud = True
		time_start = time.clock()
		print "Started..."

		setTransform()
		avoid_vectors = []

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

				if p[0] == None or p[1] == None or p[2] == None:
					cloud_out[row][col] = [None, None, None, 0]
				else:
					p2 = quaternion_multiply(quaternion_multiply(r1, p1), r2)
					p2 = [p2[1], p2[2], p2[3]]
					p2 = np.add(p2, t1)
					cloud_out[row][col] = [p2[0], p2[1], p2[2], 0]

					dist = math.sqrt(((0-p2[0])**2)+((0-p2[1])**2))
					if should_avoid(p2[0], p2[1], p2[2], dist):
						x = -1.0 / dist * p2[0] * av_coef
						y = -1.0 / dist * p2[1] * av_coef
						avoid_vectors.append([x, y])
						
		if len(avoid_vectors) > 0:
			avoid_sum = [0, 0]
			for v in avoid_vectors:
				avoid_sum[0] = avoid_sum[0] + v[0]
				avoid_sum[1] = avoid_sum[1] + v[1]
				
			compute_trajectory(avoid_sum[0], avoid_sum[1])
		else:
			tr2_b0_pub.publish(4.0)
			tr2_b1_pub.publish(4.0)

		#print cloud_in[45][0]
		#print cloud_out[45][0]

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
	tr2_b0_pub = rospy.Publisher("/tr2/joints/b0/control/velocity", Float64, queue_size=10);
	tr2_b1_pub = rospy.Publisher("/tr2/joints/b1/control/velocity", Float64, queue_size=10);
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


