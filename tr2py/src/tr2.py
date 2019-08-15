#!/usr/bin/env python

import sys
import time
import signal
import math
import tr2_msgs
from tr2_joint import Joint

CMD_SET_MODE = 0x10
CMD_SET_POS = 0x11
CMD_RESET_POS = 0x12
CMD_ROTATE = 0x13
CMD_RETURN_STATUS = 0x14
CMD_STOP_RELEASE = 0x15
CMD_STOP_EMERGENCY = 0x16

TAU = math.pi * 2.0

close = False
def signal_handler(sig, frame):
	global close
	close = True
	sys.exit(0)
	
signal.signal(signal.SIGINT, signal_handler)

class TR2:
	_msgs = tr2_msgs.Msgs()

	_state = None
	state_change = None

	mode_servo = 0x10
	mode_backdrive = 0x11
	mode_rotate = 0x12

	def __init__(self):
		self.a0 = Joint(self, "a0")
		self.a1 = Joint(self, "a1")
		self.a2 = Joint(self, "a2")
		self.a3 = Joint(self, "a3")
		self.a4 = Joint(self, "a4")
		self.h0 = Joint(self, "h0")
		self.h1 = Joint(self, "h1")
		self._msgs.state_change = self.handle_state_change

	def handle_state_change(self, state):
		if self.state_change != None:
			self.state_change(state)
			
	def state(self):
		return self._state
		
	def drive(self, motorLeft, motorRight, motorDuration = 250):
		offsetBinary = 100
			
		packet = tr2_msgs.Packet()
		packet.address = "b0"
		packet.cmd = CMD_ROTATE
		packet.addParam(int((motorLeft * 100.0) + offsetBinary))
		packet.addParam(int((motorRight * 100.0) + offsetBinary))
		packet.addParam(int(math.floor(motorDuration % 256)))
		packet.addParam(int(math.floor(motorDuration / 256)))
		
		self._msgs.add(packet)
		self.step()

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
		global close
		if close == True:
			self._msgs.close()
			self.close()
			return

		self._msgs.step()
		self._state = self._msgs.state()

		ids, states = self._msgs.state()
		for i in range(len(ids)):
			try:
				getattr(self,ids[i])._state = states[i]
			except:
				pass
		
	def spin(self, condition = True):
		global close
		print "TR2 Ready"
		while condition == True or close == True:
			self.step()
		self.close()
			
	def close(self):
		pass
