#ifndef TR2CPP__SEGMENT_H
#define TR2CPP__SEGMENT_H

#include <tr2cpp/joint.h>

namespace tr2cpp
{
	template <int T> class Segment
	{
		private:

		public:
			Segment() { };
			~Segment() { };
			int size() const { return T; }
			Joint joints[T];
	};
}

#endif
