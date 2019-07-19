#include "ros/ros.h"
#include <stdexcept>
#include <tr2cpp/tr2.h>
#include <tr2cpp/joint.h>

namespace tr2cpp
{
	TR2::TR2()
	{
		joints[0].name = "b0";
		joints[1].name = "a0";
		joints[2].name = "a1";
		joints[3].name = "a2";
		joints[4].name = "a3";
		joints[5].name = "a4";
		joints[6].name = "g0";
		joints[7].name = "h0";
		joints[8].name = "h1";
	}

	TR2::~TR2()
	{

	}

	Joint TR2::getJoint(std::string jointName)
	{
		int numJoints = sizeof(joints) / sizeof(joints[0]);
		for (int i = 0; i < numJoints; i++)
		{
			if (joints[i].name == jointName)
			{
				return joints[i];
			}
		}

		throw std::runtime_error("Could not find joint with name " + jointName);
	}

	void TR2::step()
	{
		_msgs.step();
	}

	void TR2::setJoint(Joint joint)
	{
		bool foundJoint = false;

		int numJoints = sizeof(joints) / sizeof(joints[0]);
		for (int i = 0; i < numJoints; i++)
		{
			if (joints[i].name == joint.name)
			{
				foundJoint = true;
				joints[i] = joint;
			}
		}

		if (foundJoint == false)
		{
			throw std::runtime_error("Could not find joint with name " + joint.name);
		}
	}
}
