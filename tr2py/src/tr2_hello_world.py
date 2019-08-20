#!/usr/bin/env python

from tr2py.tr2 import TR2

tr2 = TR2()
tr2.setMode(tr2.mode_servo)
tr2.release()

a0_state = tr2.a0.state()
a2_state = tr2.a2.state()
a4_state = tr2.a4.state()

tr2.a0.setPosition(a0_state + 0.25)
tr2.a2.setPosition(a2_state + 0.48)
tr2.a4.setPosition(a4_state - 0.15)
tr2.sleep(5)

tr2.a0.setPosition(a0_state)
tr2.a2.setPosition(a2_state)
tr2.a4.setPosition(a4_state)
tr2.sleep(5)

tr2.stop()
tr2.close()
