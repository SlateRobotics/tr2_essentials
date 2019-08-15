#!/usr/bin/env python

import sys
import time
import signal
import math
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_msgs.msg import UInt8
from std_msgs.msg import Float64

class Joint:
	_tr2 = None
	_id = ""
	_state = None

	_pub_pos = None
	_pub_effort = None
	_pub_stop = None
	_pub_mode = None

	def __init__(self, t, i):
		self._tr2 = t
		self._id = i

		_topic = "/tr2/joints/" + self._id;
		self._pub_stop = rospy.Publisher(_topic + "/stop", Bool, queue_size=10)
		self._pub_mode = rospy.Publisher(_topic + "/mode", UInt8, queue_size=10)
		self._pub_pos = rospy.Publisher(_topic + "/control/position", Float64, queue_size=10)
		self._pub_effort = rospy.Publisher(_topic + "/control/effort", Float64, queue_size=10)

	def state(self):
		return self._state

	def release(self):
		self._pub_stop.publish(0)

	def actuate(self, m, motorDuration = 250):
		self._pub_effort.publish(m)

	def setPosition(self, p):
		self._pub_pos.publish(p)

	def stop(self):
		self._pub_stop.publish(1)

	def setMode(self, mode):
		m = 0
		if (mode == TR2.mode_backdrive):
			m = 1
		if (mode == TR2.mode_servo):
			m = 2

		self._pub_mode.publish(m)

class TR2:
	_state = None
	state_change = None

	mode_servo = 0x10
	mode_backdrive = 0x11
	mode_rotate = 0x12

	def __init__(self):
		rospy.init_node('tr2_sim', anonymous=True)
		rospy.Subscriber("/tr2/state", JointState, self._cbtr2state)

		self.b0 = Joint(self, "b0")
		self.b1 = Joint(self, "b1")
		self.a0 = Joint(self, "a0")
		self.a1 = Joint(self, "a1")
		self.a2 = Joint(self, "a2")
		self.a3 = Joint(self, "a3")
		self.a4 = Joint(self, "a4")
		self.g0 = Joint(self, "g0")
		self.h0 = Joint(self, "h0")
		self.h1 = Joint(self, "h1")

		print("TR2 waiting for ROS state on /tr2/state")
		time.sleep(2)
		while self._state == None:
			pass

		print("TR2 ready")

	def _cbtr2state(self, msg):
		state = [msg.name, []]
		for i in range(len(state[0])):
			id = msg.name[i]
			pos = msg.position[i]
			state[1].append(msg.position[i])

			try:
				getattr(self,id)._state = pos
			except:
				pass

		self._state = state

		if self.state_change != None:
			self.state_change(self._state)
			
	def state(self):
		return self._state
		
	def drive(self, motorLeft, motorRight, motorDuration = 250):
		self.b0.actuate(motorLeft)
		self.b1.actuate(motorRight)

	def release(self):
		self.a0.release()
		self.a1.release()
		self.a2.release()
		self.a3.release()
		self.a4.release()
		self.h0.release()
		self.h1.release()
		
	def stop(self):
		self.a0.stop()
		self.a1.stop()
		self.a2.stop()
		self.a3.stop()
		self.a4.stop()
		self.h0.stop()
		self.h1.stop()
		
	def setMode(self, mode):
		self.a0.setMode(mode)
		self.a1.setMode(mode)
		self.a2.setMode(mode)
		self.a3.setMode(mode)
		self.a4.setMode(mode)
		self.h0.setMode(mode)
		self.h1.setMode(mode)
    
	def step(self):
		pass
		
	def spin(self, condition = True):
		global close
		print "TR2 Ready"
		while condition == True or close == True:
			self.step()
		self.close()
			
	def close(self):
		pass