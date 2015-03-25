#include "teleop.h"

#include "termios.h"
#include "signal.h"

#include "boost/thread/thread.hpp"

int kfd = 0;
struct termios cooked, raw;

#include "ros/ros.h"

void quit(int sig)
{
	tcsetattr(kfd, TCSANOW, &cooked);
	exit(0);
}

int main( int argc, char** argv )
{
	ros::init( argc, argv, "teleop_node" );
	ros::NodeHandle n_;

	signal(SIGINT,quit);

	RosAriaTeleop kt( cooked, raw, kfd );

	boost::thread my_thread(boost::bind(&RosAriaTeleop::spin, &kt));
	ros::Timer timer =
			n_.createTimer(ros::Duration(0.1), boost::bind(&RosAriaTeleop::watchdog, &kt));

	ros::spin();

	my_thread.interrupt();
	my_thread.join();

	return 0;
}
