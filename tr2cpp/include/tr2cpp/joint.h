#ifndef TR2CPP__JOINT_H
#define TR2CPP__JOINT_H

#include <sstream>
#include <tr2cpp/msgs.h>

namespace tr2cpp
{
	class Joint
	{
		private:
			double _previousEffort;
		public:
			std::string name;
			Joint();
			~Joint();
			void setMode(int mode);
			void actuate(double effort, uint8_t duration);
			void setPosition(double pos);
			double getPosition();
			double getPreviousEffort();
	};
}

#endif
