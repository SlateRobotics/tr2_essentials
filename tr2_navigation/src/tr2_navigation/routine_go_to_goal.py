#!/usr/bin/env python

import sys
import time
import signal
import math

class go_to_goal:
    priority = 3

    def __init__(self):
        pass

    def flag(self, cloud):
        return True

    def step(self):
        return (8.8, 0)
