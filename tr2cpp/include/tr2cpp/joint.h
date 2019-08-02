#ifndef TR2CPP__JOINT_H
#define TR2CPP__JOINT_H

#include <sstream>
#include <tr2cpp/msgs.h>

namespace tr2cpp
{
	class Joint
	{
		private:
			double _previousEffort = 0;
		public:
			Joint();
			~Joint();

			Msgs *_msgs;

			std::string name = "x0";
			double pos = 0;

			void setMode(int mode);
			void actuate(double effort, int duration);
			void setPosition(double pos, double speed);
			double getPosition();
			double getPreviousEffort();
	};
}

#endif
