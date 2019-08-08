
#ifndef TR2CPP__MSG_H
#define TR2CPP__MSG_H

#include "packet.h"

namespace tr2cpp
{
	class Msg
	{
		private:
			Packet packet;

		public:
			uint64_t expires;
			uint64_t _expirePeriod = 500;

			Msg() { }

			Msg(Packet p)
			{
				packet = p;
			}

			~Msg() { }

			int id()
			{
				return packet.msgId;
			}

			void step()
			{
			}
	};
}

#endif
