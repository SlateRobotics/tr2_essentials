#include "ros/ros.h"
#include <stdexcept>
#include <tr2cpp/tr2.h>
#include <tr2cpp/segment.h>
#include <tr2cpp/joint.h>

namespace tr2cpp
{
	TR2::TR2()
	{
		//base
		base.joints[0].name = "JointBaseWheelL";
		base.joints[0].setMotorId(6);
		base.joints[1].name = "JointBaseWheelR";
		base.joints[1].setMotorId(7);

		//head
		head.joints[0].name = "JointHeadPan";
		head.joints[0].setMotorId(8);
		head.joints[1].name = "JointHeadTilt";
		head.joints[1].setMotorId(9);

		//arm
		arm.joints[0].name = "JointArm0";
		arm.joints[0].setMotorId(0);
		arm.joints[1].name = "JointArm1";
		arm.joints[1].setMotorId(1);
		arm.joints[2].name = "JointArm2";
		arm.joints[2].setMotorId(2);
		arm.joints[3].name = "JointArm3";
		arm.joints[3].setMotorId(3);
		arm.joints[4].name = "JointArm4";
		arm.joints[4].setMotorId(4);
		arm.joints[5].name = "JointArmGripper";
		arm.joints[5].setMotorId(5);
		arm.joints[5].setActuatorType(ACTUATOR_TYPE_SERVO);

	}

	TR2::~TR2()
	{

	}

	Joint TR2::getJoint(std::string jointName)
	{
		int numJointsHead = sizeof(head.joints) / sizeof(head.joints[0]);
		for (int i = 0; i < numJointsHead; i++)
		{
			if (head.joints[i].name == jointName)
			{
				return head.joints[i];
			}
		}

		int numJointsBase = sizeof(base.joints) / sizeof(base.joints[0]);
		for (int i = 0; i < numJointsBase; i++)
		{
			if (base.joints[i].name == jointName)
			{
				return base.joints[i];
			}
		}

		int numJointsArm = sizeof(arm.joints) / sizeof(arm.joints[0]);
		for (int i = 0; i < numJointsArm; i++)
		{
			if (arm.joints[i].name == jointName)
			{
				return arm.joints[i];
			}
		}

		throw std::runtime_error("Could not find joint with name " + jointName);
	}

	void TR2::setJoint(Joint joint)
	{
		bool foundJoint = false;

		int numJointsHead = sizeof(head.joints) / sizeof(head.joints[0]);
		for (int i = 0; i < numJointsHead; i++)
		{
			if (head.joints[i].name == joint.name)
			{
				foundJoint = true;
				head.joints[i] = joint;
			}
		}

		int numJointsBase = sizeof(base.joints) / sizeof(base.joints[0]);
		for (int i = 0; i < numJointsBase; i++)
		{
			if (base.joints[i].name == joint.name)
			{
				foundJoint = true;
				base.joints[i] = joint;
			}
		}

		int numJointsArm = sizeof(arm.joints) / sizeof(arm.joints[0]);
		for (int i = 0; i < numJointsArm; i++)
		{
			if (arm.joints[i].name == joint.name)
			{
				foundJoint = true;
				arm.joints[i] = joint;
			}
		}

		if (foundJoint == false)
		{
			throw std::runtime_error("Could not find joint with name " + joint.name);
		}
	}
}
