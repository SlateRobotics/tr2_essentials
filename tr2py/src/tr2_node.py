#!/usr/bin/env python

import time
import sys
import math
from tr2 import TR2
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_msgs.msg import UInt8
from std_msgs.msg import Float64

tr2 = TR2()

tr2_state_pub = None
tr2_state_a0_pub = None
tr2_state_a1_pub = None
tr2_state_a2_pub = None
tr2_state_a3_pub = None
tr2_state_a4_pub = None
tr2_state_g0_pub = None
tr2_state_h0_pub = None
tr2_state_h1_pub = None

def change_mode(d, j = -1):
	b = int(d)
	mode = tr2.mode_rotate
	if b == 1:
		mode = tr2.mode_backdrive
	elif b == 2:
		mode = tr2.mode_servo

	if j == -1:
		tr2.setMode(mode)
	else:
		j.setMode(mode)

def stop(d, j = -1):
	b = bool(d)
	if b == True:
		if j == -1:
			tr2.stop()
		else:
			j.stop()
	else:
		if j == -1:
			tr2.release()
		else:
			j.release()

def ctrl_pos(d, j):
	j.setPosition(d)

def ctrl_effort(d, j):
	j.actuate(d)

def mode_tr2(msg):
	change_mode(msg.data)

def mode_tr2_servo(msg):
	b = bool(msg.data)
	if b == True:
		tr2.setMode(tr2.mode_servo)
	else:
		tr2.setMode(tr2.mode_rotate)

def mode_tr2_backdrive(msg):
	b = bool(msg.data)
	if b == True:
		tr2.setMode(tr2.mode_backdrive)
	else:
		tr2.setMode(tr2.mode_rotate)

def mode_tr2_rotate(msg):
	b = bool(msg.data)
	if b == True:
		tr2.setMode(tr2.mode_rotate)
	else:
		tr2.setMode(tr2.mode_rotate)

def mode_a0(msg):
	change_mode(msg.data, tr2.a0)

def mode_a1(msg):
	change_mode(msg.data, tr2.a1)

def mode_a2(msg):
	change_mode(msg.data, tr2.a2)

def mode_a3(msg):
	change_mode(msg.data, tr2.a3)

def mode_a4(msg):
	change_mode(msg.data, tr2.a4)

def mode_g0(msg):
	change_mode(msg.data, tr2.g0)

def mode_h0(msg):
	change_mode(msg.data, tr2.h0)

def mode_h1(msg):
	change_mode(msg.data, tr2.h1)

def tr2_stop(msg):
	stop(msg.data)

def tr2_a0_stop(msg):
	stop(msg.data, tr2.a0)

def tr2_a1_stop(msg):
	stop(msg.data, tr2.a1)

def tr2_a2_stop(msg):
	stop(msg.data, tr2.a2)

def tr2_a3_stop(msg):
	stop(msg.data, tr2.a3)

def tr2_a4_stop(msg):
	stop(msg.data, tr2.a4)

def tr2_g0_stop(msg):
	stop(msg.data, tr2.g0)

def tr2_h0_stop(msg):
	stop(msg.data, tr2.h0)

def tr2_h1_stop(msg):
	stop(msg.data, tr2.h1)

def ctrl_pos_a0(msg):
	ctrl_pos(msg.data, tr2.a0)

def ctrl_pos_a1(msg):
	ctrl_pos(msg.data, tr2.a1)

def ctrl_pos_a2(msg):
	ctrl_pos(msg.data, tr2.a2)

def ctrl_pos_a3(msg):
	ctrl_pos(msg.data, tr2.a3)

def ctrl_pos_a4(msg):
	ctrl_pos(msg.data, tr2.a4)

def ctrl_pos_g0(msg):
	ctrl_pos(msg.data, tr2.g0)

def ctrl_pos_h0(msg):
	ctrl_pos(msg.data, tr2.h0)

def ctrl_pos_h1(msg):
	ctrl_pos(msg.data, tr2.h1)

def ctrl_effort_a0(msg):
	ctrl_effort(msg.data, tr2.a0)

def ctrl_effort_a1(msg):
	ctrl_effort(msg.data, tr2.a1)

def ctrl_effort_a2(msg):
	ctrl_effort(msg.data, tr2.a2)

def ctrl_effort_a3(msg):
	ctrl_effort(msg.data, tr2.a3)

def ctrl_effort_a4(msg):
	ctrl_effort(msg.data, tr2.a4)

def ctrl_effort_h0(msg):
	ctrl_effort(msg.data, tr2.h0)

def ctrl_effort_h1(msg):
	ctrl_effort(msg.data, tr2.h1)

append_states = False
def tr2_state_change(state):
	global tr2_state_pub, tr2_state_a0_pub, tr2_state_a1_pub, tr2_state_a2_pub, tr2_state_a3_pub, tr2_state_a4_pub, tr2_state_g0_pub, tr2_state_h0_pub, tr2_state_h1_pub, append_states

	if append_states == False:
		state[0].append('g0_b')
		state[1].append(0.0)
		append_states = True

	joint_state = JointState()
	joint_state.name = state[0]
	joint_state.position = state[1]

	g0_pos = 0.0
	for i in range(len(joint_state.name)):
		if joint_state.name[i] == 'g0':
			joint_state.position[i] = joint_state.position[i] * 0.041
			g0_pos = joint_state.position[i]

	for i in range(len(joint_state.name)):
		if joint_state.name[i] == 'g0_b':
			joint_state.position[i] = -g0_pos

	tr2_state_pub.publish(joint_state)

	for i in range(len(state[0])):
		id, s = state[0][i], state[1][i]
		p = None

		if id == 'a0':
			p = tr2_state_a0_pub
		elif id == 'a1':
			p = tr2_state_a1_pub
		elif id == 'a2':
			p = tr2_state_a2_pub
		elif id == 'a3':
			p = tr2_state_a3_pub
		elif id == 'a4':
			p = tr2_state_a4_pub
		elif id == 'g0':
			p = tr2_state_g0_pub
		elif id == 'h0':
			p = tr2_state_h0_pub
		elif id == 'h1':
			p = tr2_state_h1_pub

		if p != None:
			p.publish(s)

def program():
	global close, tr2_state_pub, tr2_state_a0_pub, tr2_state_a1_pub, tr2_state_a2_pub, tr2_state_a3_pub, tr2_state_a4_pub, tr2_state_g0_pub, tr2_state_h0_pub, tr2_state_h1_pub

	rospy.init_node('tr2_node', anonymous=True)
	rospy.Subscriber("/tr2/mode", UInt8, mode_tr2)
	rospy.Subscriber("/tr2/mode/servo", Bool, mode_tr2_servo)
	rospy.Subscriber("/tr2/mode/backdrive", Bool, mode_tr2_backdrive)
	rospy.Subscriber("/tr2/mode/rotate", Bool, mode_tr2_rotate)
	rospy.Subscriber("/tr2/joints/a0/mode", UInt8, mode_a0)
	rospy.Subscriber("/tr2/joints/a1/mode", UInt8, mode_a1)
	rospy.Subscriber("/tr2/joints/a2/mode", UInt8, mode_a2)
	rospy.Subscriber("/tr2/joints/a3/mode", UInt8, mode_a3)
	rospy.Subscriber("/tr2/joints/a4/mode", UInt8, mode_a4)
	rospy.Subscriber("/tr2/joints/g0/mode", UInt8, mode_g0)
	rospy.Subscriber("/tr2/joints/h0/mode", UInt8, mode_h0)
	rospy.Subscriber("/tr2/joints/h1/mode", UInt8, mode_h1)
	rospy.Subscriber("/tr2/stop", Bool, tr2_stop)
	rospy.Subscriber("/tr2/joints/a0/stop", Bool, tr2_a0_stop)
	rospy.Subscriber("/tr2/joints/a1/stop", Bool, tr2_a1_stop)
	rospy.Subscriber("/tr2/joints/a2/stop", Bool, tr2_a2_stop)
	rospy.Subscriber("/tr2/joints/a3/stop", Bool, tr2_a3_stop)
	rospy.Subscriber("/tr2/joints/a4/stop", Bool, tr2_a4_stop)
	rospy.Subscriber("/tr2/joints/g0/stop", Bool, tr2_g0_stop)
	rospy.Subscriber("/tr2/joints/h0/stop", Bool, tr2_h0_stop)
	rospy.Subscriber("/tr2/joints/h1/stop", Bool, tr2_h1_stop)
	rospy.Subscriber("/tr2/joints/a0/control/position", Float64, ctrl_pos_a0)
	rospy.Subscriber("/tr2/joints/a1/control/position", Float64, ctrl_pos_a1)
	rospy.Subscriber("/tr2/joints/a2/control/position", Float64, ctrl_pos_a2)
	rospy.Subscriber("/tr2/joints/a3/control/position", Float64, ctrl_pos_a3)
	rospy.Subscriber("/tr2/joints/a4/control/position", Float64, ctrl_pos_a4)
	rospy.Subscriber("/tr2/joints/g0/control/position", Float64, ctrl_pos_g0)
	rospy.Subscriber("/tr2/joints/h0/control/position", Float64, ctrl_pos_h0)
	rospy.Subscriber("/tr2/joints/h1/control/position", Float64, ctrl_pos_h1)
	rospy.Subscriber("/tr2/joints/a0/control/effort", Float64, ctrl_effort_a0)
	rospy.Subscriber("/tr2/joints/a1/control/effort", Float64, ctrl_effort_a1)
	rospy.Subscriber("/tr2/joints/a2/control/effort", Float64, ctrl_effort_a2)
	rospy.Subscriber("/tr2/joints/a3/control/effort", Float64, ctrl_effort_a3)
	rospy.Subscriber("/tr2/joints/a4/control/effort", Float64, ctrl_effort_a4)
	rospy.Subscriber("/tr2/joints/h0/control/effort", Float64, ctrl_effort_h0)
	rospy.Subscriber("/tr2/joints/h1/control/effort", Float64, ctrl_effort_h1)

	tr2_state_pub = rospy.Publisher("/tr2/state", JointState, queue_size=1)
	tr2_state_a0_pub = rospy.Publisher("/tr2/joints/a0/state", Float64, queue_size=1)
	tr2_state_a1_pub = rospy.Publisher("/tr2/joints/a1/state", Float64, queue_size=1)
	tr2_state_a2_pub = rospy.Publisher("/tr2/joints/a2/state", Float64, queue_size=1)
	tr2_state_a3_pub = rospy.Publisher("/tr2/joints/a3/state", Float64, queue_size=1)
	tr2_state_a4_pub = rospy.Publisher("/tr2/joints/a4/state", Float64, queue_size=1)
	tr2_state_g0_pub = rospy.Publisher("/tr2/joints/g0/state", Float64, queue_size=1)
	tr2_state_h0_pub = rospy.Publisher("/tr2/joints/h0/state", Float64, queue_size=1)
	tr2_state_h1_pub = rospy.Publisher("/tr2/joints/h1/state", Float64, queue_size=1)

	tr2.state_change = tr2_state_change
	tr2.spin()

if __name__ == '__main__':
	program()
