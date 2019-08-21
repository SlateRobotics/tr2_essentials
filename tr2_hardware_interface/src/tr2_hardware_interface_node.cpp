#include <ros/callback_queue.h>
#include <tr2_hardware_interface/tr2_hardware_interface.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tr2_hardware_interface");
	ros::CallbackQueue ros_queue;

  ros::NodeHandle nh;
	nh.setCallbackQueue(&ros_queue);
  tr2_hardware_interface::TR2HardwareInterface tr2(nh);

	ros::MultiThreadedSpinner spinner(0);
	spinner.spin(&ros_queue);

  return 0;
}
