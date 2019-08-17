#!/usr/bin/env python

import time
from tr2py.tr2_sim import TR2

tr2 = TR2()
tr2.setMode(tr2.mode_servo)
tr2.release()

poses = [[0,0,0,0,0,0,4],[0,0,0,1.5,0,0,4],[0,0,0.7,1.5,0,1,4],[0,0,-0.7,1.5,0,1,8],[0,0,0,1.5,0,0,4]]

tr2.drive(0.10, -0.10)
for p in poses:
	print("Moving to position", p)
	tr2.a0.setPosition(p[0])
	tr2.a1.setPosition(p[1])
	tr2.a2.setPosition(p[2])
	tr2.a3.setPosition(p[3])
	tr2.a4.setPosition(p[4])
	tr2.g0.setPosition(p[5])
	time.sleep(p[6])

tr2.drive(0, 0)
tr2.stop()
