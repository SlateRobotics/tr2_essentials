#!/usr/bin/env python

import curses
import time
import rospy
import sys
import signal
import numpy as np
import math
import datetime
from tr2py.tr2 import TR2

tr2 = TR2()
tr2.setMode(tr2.mode_rotate)
tr2.release()

jointSelected = 0;
joints = ["b0", "a0", "a1", "a2", "a3", "a4", "g0", "h0", "h1"]

modeSelected = 0
modes = ["ROTATE","BACKDRIVE","SERVO"]

stop = "FALSE"

def program():
	global jointSelected, joints, modeSelected, modes, stop, tr2

	rospy.init_node('tr2_keyboard_teleop', anonymous=True)

	screen = curses.initscr()
	curses.noecho()
	curses.cbreak()
	screen.keypad(True)
	screen.nodelay(1)

	cmd = 0
	prevMode = modeSelected

	try:
		while True:
			char = screen.getch()

			if char == ord(' '): # emergency stop
				if stop == "FALSE":
					stop = "TRUE"
					tr2.stop()
				else:
					stop = "FALSE"
					tr2.release()
			elif char == ord('q'): break
			elif char == ord('d'): modeSelected = modeSelected + 1
			elif char == ord('a'): modeSelected = modeSelected - 1
			elif char == curses.KEY_UP: jointSelected = jointSelected + 1
			elif char == curses.KEY_DOWN: jointSelected = jointSelected - 1
			elif char == curses.KEY_LEFT: cmd = -1
			elif char == curses.KEY_RIGHT: cmd = 1
			else:
				cmd = 0

			aid = joints[jointSelected]

			if modeSelected != prevMode:
				if modeSelected == 0:
					getattr(tr2, aid).setMode(tr2.mode_rotate)
				elif modeSelected == 1:
					getattr(tr2, aid).setMode(tr2.mode_backdrive)
				elif modeSelected == 2:
					getattr(tr2, aid).setMode(tr2.mode_servo)

			if modeSelected == 0:
				if (cmd != 0):
					getattr(tr2, aid).actuate(cmd, 100)
			elif modeSelected == 2:
					getattr(tr2, aid).setPosition(cmd, 100)

			tr2.step()

			screen.clrtobot()
			screen.addstr(0,0,"Welcome. Use arrow keys to control. Press 'Q' to exit, SPACE to emergency stop the robot.")
			screen.addstr(1,0,"ACTUATOR: " + aid)
			screen.addstr(2,0,"MODE: " + modes[modeSelected])
			screen.addstr(3,0,"CMD: " + str(cmd) + "   ")
			screen.addstr(4,0,"STOP: " + stop)
			screen.addstr(5,0,"INPUT: " + str(char))
			screen.refresh()

			curses.flushinp()
			time.sleep(0.05)
	finally:
		screen.addstr(0,0,"exit")
		curses.nocbreak()
		screen.keypad(0)
		curses.echo()
		curses.endwin()

if __name__ == '__main__':
	program()
