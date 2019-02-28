#include <stdlib.h>
#include <math.h>
#include <stdexcept>
#include "ros/ros.h"
#include <tr2cpp/joint.h>
#include <tr2cpp/serial.h>

#define PI 3.14159265359
#define TAU 6.28318530718

namespace tr2cpp
{
	Joint::Joint()
	{
		char serialName[13] = "/dev/ttyACM0";
		this->_serialPort = new SerialPort(serialName);
	}

	Joint::Joint(uint8_t motorId)
	{
		setMotorId(motorId);
	}

	Joint::~Joint()
	{

	}

	void Joint::setActuatorType(uint8_t actuatorType)
	{
		this->_actuatorType = actuatorType;
	}

	uint8_t Joint::getMotorId()
	{
		return this->_motorId;
	}

	void Joint::setMotorId(uint8_t motorId)
	{
		this->_motorId = motorId;
	}

	double Joint::_filterAngle(double angle)
	{
		_angleReads = _angleReads + 1;

		// put value at front of array
		for (int i = _filterPrevious - 1; i > 0; i--) {
			_previousAngles[i] = _previousAngles[i - 1];
		}
		_previousAngles[0] = angle;


		int filterIterations = _filterPrevious;
		if (_angleReads < _filterPrevious) {
			filterIterations = _angleReads;
		}

		double angleSum = 0;
		for (int i = 0; i < filterIterations; i++) {
			angleSum = angleSum + _previousAngles[i];
		}

		double filterResult = angleSum / (filterIterations * 1.0);

		//ROS_INFO("%f, %f, %f, %i", angle, angleSum, filterResult, filterIterations);

		return filterResult;
	}

	double Joint::readAngle()
	{
		if (_actuatorType == ACTUATOR_TYPE_MOTOR) {
			/*uint16_t position;
			I2C i2cSlave = I2C(1, _getSlaveAddress());
			uint8_t result = i2cSlave.readBytes(_motorId, 4, position);
			if (result == 1) {
				double angle = (position / sensorResolution * TAU);
				angle = _filterAngle(angle);
				angle += angleOffset;
				if (angle > PI) angle -= TAU;
				if (angle < -PI) angle += TAU;
				angle *= readRatio;
				return angle;
			} else {
				//throw std::runtime_error("I2C Read Error during joint position read. Exiting for safety.");
			}*/
			return 0;
		}
		else if (_actuatorType == ACTUATOR_TYPE_SERVO)
		{
			return _previousEffort;
		}
		else
		{
			return 0;
		}
	}

	void Joint::setPosition(double pos)
	{
		uint8_t packetSize = 5;
		uint8_t packet[packetSize];
		packet[0] = 255;
		packet[1] = packetSize;
		packet[2] = CMD_SET_POS;
		packet[3] = mode;
		packet[3] = mode;
		
		long checksum = 0;
		for (int i = 0; i < packetSize - 1; i++) {
			checksum += packet[i];
		}
		packet[4] = checksum % 256;

		char data[255];
		snprintf(data, 255, "0x%x %i %i %i %i %i ;", _getSlaveAddress(), packet[0], packet[1], packet[2], packet[3], packet[4]);
		bool success = _serialPort->writeSerialPort(data);
		if (!success) {
			ROS_WARN("Could not write to serial port");
		}
	}

	void Joint::setMode(int mode)
	{
		if (_serialPort->isConnected()) {
			uint8_t packetSize = 5;
			uint8_t packet[packetSize];
			packet[0] = 255;
			packet[1] = packetSize;
			packet[2] = CMD_SET_MODE;
			packet[3] = mode;
		
			long checksum = 0;
			for (int i = 0; i < packetSize - 1; i++) {
				checksum += packet[i];
			}
			packet[4] = checksum % 256;

			char data[255];
			snprintf(data, 255, "0x%x %i %i %i %i %i ;", _getSlaveAddress(), packet[0], packet[1], packet[2], packet[3], packet[4]);
			bool success = _serialPort->writeSerialPort(data);
			if (!success) {
				ROS_WARN("Could not write to serial port");
			}
		}
	}

	void Joint::actuate(double effort, uint8_t duration = 15)
	{
		if (effort > 1.0) effort = 1.0;
		if (effort < -1.0) effort = -1.0;
		if (abs(effort * 100.0) < 20) return; // because it's too little to do anything

		if (_serialPort->isConnected()) {
			int offsetBinary = 128;
			uint8_t packetSize = 5;
			uint8_t packet[packetSize];
			packet[0] = 255;
			packet[1] = packetSize;
			packet[2] = CMD_ROTATE;
			packet[3] = floor(effort * 100) + offsetBinary;
			
			long checksum = 0;
			for (int i = 0; i < packetSize - 1; i++) {
				checksum += packet[i];
			}
			packet[4] = checksum % 256;

			char data[255];
			snprintf(data, 255, "0x%x %i %i %i %i %i ;", _getSlaveAddress(), packet[0], packet[1], packet[2], packet[3], packet[4]);
			bool success = _serialPort->writeSerialPort(data);
			if (!success) {
				ROS_WARN("Could not write to serial port");
			}
		} else {
			ROS_WARN("Serial port is not connected");
		}

		_previousEffort = effort;
	}

	uint8_t Joint::_getSlaveAddress()
	{
		if (_motorId == 0) return ARM0_SLAVE_ADDRESS;
		else if (_motorId == 1) return ARM1_SLAVE_ADDRESS;
		else if (_motorId == 2) return ARM2_SLAVE_ADDRESS;
		else if (_motorId == 3) return ARM3_SLAVE_ADDRESS;
		else if (_motorId == 4) return ARM4_SLAVE_ADDRESS;
		else if (_motorId == 5) return GRIPPER_SLAVE_ADDRESS;
		else if (_motorId == 6) return BASE_SLAVE_ADDRESS;
		else if (_motorId == 7) return BASE_SLAVE_ADDRESS;
		else if (_motorId == 8) return HEAD0_SLAVE_ADDRESS;
		else if (_motorId == 9) return HEAD1_SLAVE_ADDRESS;
		else
		{
			ROS_ERROR("Invalid MotorID: %i", _motorId);
			return -1;
		}
	}

	void Joint::setServoLimits(uint8_t minValue, uint8_t maxValue)
	{
		this->_minServoValue = minValue;
		this->_maxServoValue = maxValue;
	}

	double Joint::getPreviousEffort() {
		return this->_previousEffort;
	}

	void Joint::_prepareI2CWrite(uint8_t result[4], double effort)
	{
		if (_actuatorType == ACTUATOR_TYPE_MOTOR)
		{
			if (effort > 1.0) effort = 1.0;
			if (effort < -1.0) effort = -1.0;
			uint8_t speed = floor(abs(effort * 100));
			uint8_t direction = (effort > 0);
			//uint8_t duration = 5;

			result[0] = _motorId;
			result[1] = speed;
			result[2] = direction;
			//result[3] = duration;
		}
		else if (_actuatorType == ACTUATOR_TYPE_SERVO)
		{
			/*if (name != "JointRightGripper") {
				effort = (effort + 1.5708) / 3.1415;
				if (effort > 1.0) effort = 1.0;
				if (effort < 0.0) effort = 0.0;
			}*/

			double magnitude = effort * 100.0;
			uint8_t servoValue = floor(_minServoValue + ((_maxServoValue - _minServoValue) * (magnitude / 100.0)));

			result[0] = _motorId;
			result[1] = servoValue;
			result[2] = 0;
			result[3] = 0;

			//ROS_INFO("name: %s, minServoValue: %i, maxServoValue: %i, effort: %f, magnitude: %f, servoValue: %i", name.c_str(), _minServoValue, _maxServoValue, effort, magnitude, servoValue);
		}
	}
	
	int Joint::getActuatorType()
	{
		return _actuatorType;
	}
}
