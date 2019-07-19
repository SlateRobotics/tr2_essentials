#ifndef TR2CPP__MSGS_H
#define TR2CPP__MSGS_H

#define CMD_SET_MODE 0x10
#define CMD_SET_POS 0x11
#define CMD_RESET_POS 0x12
#define CMD_ROTATE 0x13
#define CMD_RETURN_STATUS 0x14
#define CMD_SET_FREQUENCY 0x15

#define MODE_SERVO 0x10
#define MODE_BACKDRIVE 0x11
#define MODE_ROTATE 0x12

#include "Socket.h"
#include "ProtocolSimple.h"
#include <iostream>

namespace Sock = ThorsAnvil::Socket;

namespace tr2cpp
{
	class Msgs
	{
		private:

		public:
			Msgs()
			{
				std::cout << "Starting server...\n";				

				Sock::ServerSocket server(12345);
				int finished = 0;
				while(!finished)
				{
					Sock::DataSocket accept = server.accept();
					Sock::ProtocolSimple acceptSimple(accept);

					acceptSimple.sendMessage("", "nc;");

					std::string message;
					message.reserve(256);
					acceptSimple.recvMessage(message);

					message = message.substr(0, message.find(";;")) + ";";

					std::cout << " > " << message << "\n";
				}

			}

			~Msgs() { }

			void read() { }
			void step() { }
			void write() { }
	};
}

#endif
