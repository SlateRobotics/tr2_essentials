#!/usr/bin/env python

import curses
import time
import rospy
from tr2py.tr2 import TR2

tr2 = TR2()
tr2.setMode(tr2.mode_rotate)
tr2.release()

JOINTS = [
    tr2.b0,
    tr2.a0,
    tr2.a1,
    tr2.a2,
    tr2.a3,
    tr2.a4,
    tr2.g0,
    tr2.h0,
    tr2.h1
]

MODES = [
    ("ROTATE", tr2.mode_rotate),
    ("BACKDRIVE", tr2.mode_backdrive),
    ("SERVO", tr2.mode_servo)
]


def get_circular_index_increment(new_index, max_index):
    if new_index > max_index:
        # If we tried to go past the end, set back to the beginning
        new_index = 0
    elif new_index < 0:
        # While python will allow negative indices, let's just reset
        # back to the end of the list to avoid confusion
        new_index = max_index
    return new_index


def program():
    rospy.init_node('tr2_keyboard_teleop', anonymous=True)

    screen = curses.initscr()
    curses.noecho()
    curses.cbreak()
    screen.keypad(True)
    screen.nodelay(1)

    joint_selected = 0
    mode_selected = 0
    prevMode = mode_selected
    max_mode_index = len(modes) - 1
    max_joint_index = len(JOINTS) - 1
    stop = False

    try:
        while True:
            char = screen.getch()
            cmd = 0

            if char == ord(' '): # emergency stop
                if not stop:
                    stop = True
                    tr2.stop()
                else:
                    stop = False
                    tr2.release()
            elif char == ord('q'):
                break
            elif char == ord('d'):
                mode_selected += 1
            elif char == ord('a'):
                mode_selected -= 1
            elif char == curses.KEY_UP:
                joint_selected += 1
            elif char == curses.KEY_DOWN:
                joint_selected -= 1
            elif char == curses.KEY_LEFT:
                cmd = -1
            elif char == curses.KEY_RIGHT:
                cmd = 1

            joint_selected = get_circular_index_increment(joint_selected)
            mode_selected = get_circular_index_increment(mode_selected)
            target_joint = JOINTS[joint_selected]

            mode_description = modes[modeSeleted][0]
            mode_action = modes[mode_selected][1]
            if mode_selected != prevMode:
                target_joint.setMode(mode_action)

            if mode_description == "ROTATE" and cmd != 0:
                target_joint.actuate(cmd, 100)
            elif mode_description == "SERVO":
                target_joint.setPosition(cmd, 100)

            tr2.step()

            screen.clrtobot()
            screen.addstr(0,0,"Welcome. Use arrow keys to control. Press 'Q' to exit, SPACE to emergency stop the robot.")
            screen.addstr(1, 0, "ACTUATOR: " + target_joint._id)
            screen.addstr(2, 0, "MODE: " + mode_description)
            screen.addstr(3, 0, "CMD: " + str(cmd) + "   ")
            screen.addstr(4, 0, "STOP: " + str(stop))
            screen.addstr(5, 0, "INPUT: " + str(char))
            screen.refresh()

            curses.flushinp()
            time.sleep(0.05)
    finally:
        screen.addstr(0, 0, "exit")
        curses.nocbreak()
        screen.keypad(0)
        curses.echo()
        curses.endwin()

if __name__ == '__main__':
    program()
