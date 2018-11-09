#ifndef SERIALPORT_H
#define SERIALPORT_H

#include <iostream>
#include <stdio.h>
using namespace std;

class SerialPort
{
	private:
		int fd;
    bool connected;
		int setup(int fd, int speed, int parity);
		void setBlocking(int fd, int should_block);
	public:
		SerialPort(char *portName);
		~SerialPort();

    int readSerialPort(char *data);
    bool writeSerialPort(std::string data);
    bool isConnected();
};

#endif // SERIALPORT_H
