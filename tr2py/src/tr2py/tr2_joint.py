#!/usr/bin/env python

import math
import tr2_msgs

CMD_SET_MODE = 0x10
CMD_SET_POS = 0x11
CMD_RESET_POS = 0x12
CMD_ROTATE = 0x13
CMD_RETURN_STATUS = 0x14
CMD_STOP_RELEASE = 0x15
CMD_STOP_EMERGENCY = 0x16
CMD_FLIP_MOTOR = 0x17

class Joint:
	_tr2 = None
	_id = None
	_state = None

	def __init__(self, t, i):
		self._tr2 = t
		self._id = i
			
	def state(self):
		return self._state
		
	def setPosition(self, pos, speed = 100):
		x = pos / (math.pi * 2) * 65535
	
		packet = tr2_msgs.Packet()
		packet.address = self._id
		packet.cmd = CMD_SET_POS
		packet.addParam(int(math.floor(x % 256)))
		packet.addParam(int(math.floor(x / 256)))
		packet.addParam(int(math.floor(speed / 100.0 * 255.0)))
		
		self._tr2._msgs.add(packet)
		self._tr2.step()

	def release(self):
		cmd = CMD_STOP_RELEASE
		
		packet = tr2_msgs.Packet()
		packet.address = self._id
		packet.cmd = cmd

		self._tr2._msgs.add(packet)
		self._tr2.step()
		
	def stop(self):
		cmd = CMD_STOP_EMERGENCY
		
		packet = tr2_msgs.Packet()
		packet.address = self._id
		packet.cmd = cmd
		
		self._tr2._msgs.add(packet)
		self._tr2.step()
		
	def actuate(self, motorValue, motorDuration = 250):
		offsetBinary = 128
		x = int(math.floor(motorValue * 100.0))
			
		packet = tr2_msgs.Packet()
		packet.address = self._id
		packet.cmd = CMD_ROTATE
		packet.addParam(x + offsetBinary)
		packet.addParam(int(math.floor(motorDuration % 256)))
		packet.addParam(int(math.floor(motorDuration / 256)))
		
		self._tr2._msgs.add(packet)
		self._tr2.step()

        def flipMotor(self):
            packet = tr2_msgs.Packet()
            packet.address = self._id
            packet.cmd = CMD_FLIP_MOTOR
            
            self._tr2._msgs.add(packet)
            self._tr2.step()
	
	def resetEncoderPosition(self):
		packet = tr2_msgs.Packet()
		packet.address = self._id
		packet.cmd = CMD_RESET_POS
		
		self._tr2._msgs.add(packet)
		self._tr2.step()
		
	def setMode(self, mode):
		packet = tr2_msgs.Packet()
		packet.address = self._id
		packet.cmd = CMD_SET_MODE
		packet.addParam(mode)
		
		self._tr2._msgs.add(packet)
		self._tr2.step()

