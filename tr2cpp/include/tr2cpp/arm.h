#ifndef TR2CPP__ARM_H
#define TR2CPP__ARM_H

#include <tr2cpp/joint.h>

namespace tr2cpp
{
	class Arm
	{
		private:

		public:
			Arm();
			~Arm();
			Joint joints[5];
	};
}

#endif
