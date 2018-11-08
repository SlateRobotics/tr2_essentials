#ifndef TR2CPP__TR2_H
#define TR2CPP__TR2_H

#include <sstream>
#include <tr2cpp/segment.h>

namespace tr2cpp
{
	class TR2
	{
		private:
		public:
			TR2();
			~TR2();

			Segment<2> base;
			Segment<2> head;
			Segment<6> arm;

			Joint getJoint(std::string jointName);
			void setJoint(tr2cpp::Joint joint);
	};
}

#endif
