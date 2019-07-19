#include <stdlib.h>
#include <math.h>
#include <stdexcept>
#include "ros/ros.h"
#include <tr2cpp/joint.h>
#include <tr2cpp/msgs.h>

#define PI 3.14159265359
#define TAU 6.28318530718

namespace tr2cpp
{
	Joint::Joint() { }
	Joint::~Joint() { }

	double Joint::getPosition()
	{
		return 0;
	}

	void Joint::setPosition(double pos)
	{
		// send msg over tcp
	}

	void Joint::setMode(int mode)
	{
		// send msg over tcp
	}

	void Joint::actuate(double effort, uint8_t duration = 15)
	{
		// send msg over tcp
		_previousEffort = effort;
	}

	double Joint::getPreviousEffort() {
		return this->_previousEffort;
	}
}
