#ifndef TR2CPP__TR2_H
#define TR2CPP__TR2_H

#include <sstream>
#include <tr2cpp/joint.h>
#include <tr2cpp/msgs.h>

namespace tr2cpp
{
	class TR2
	{
		private:
			Msgs _msgs;

		public:
			TR2();
			~TR2();

			Joint joints[10];
			Joint getJoint(std::string jointName);

			void step();
			void waitForState();
			void setJoint(tr2cpp::Joint joint);
	};
}

#endif
