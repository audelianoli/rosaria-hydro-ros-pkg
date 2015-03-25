#include "teleop.h"

RosAriaTeleop::RosAriaTeleop( struct termios &cooked, struct termios &raw, int &kfd ):
	n_(),
	cooked_( cooked ),
	raw_( raw ),
	kfd_( kfd )
{
	cmd_vel_pub_ = n_.advertise<geometry_msgs::Twist>("/RosAria/cmd_vel", 1, true);
	vel_x = 0.0;
	vel_omega = 0.0;

}

RosAriaTeleop::~RosAriaTeleop()
{
	cmd_vel_pub_.shutdown();
}

void RosAriaTeleop::publish( double vel_x, double vel_omega )
{
	cmd_vel_msg_.linear.x 	= vel_x;
	cmd_vel_msg_.angular.z 	= vel_omega;

	cmd_vel_pub_.publish( cmd_vel_msg_ );
}

void RosAriaTeleop::spin()
{
	char c;
	double vel_x, vel_y, vel_omega;

	// Get the console in raw mode
	tcgetattr( kfd_, &cooked_ );
	memcpy( &raw_, &cooked_, sizeof(struct termios) );
	raw_.c_lflag &=~ ( ICANON | ECHO );

	// Setting a new line, then end of file
	raw_.c_cc[VEOL] = 1;
	raw_.c_cc[VEOF] = 2;
	tcsetattr( kfd_, TCSANOW, &raw_ );

	puts("Reading from keyboard");
	puts("---------------------------");
	puts("Use 'WASD' for translation");

	puts("Press 'Space' to STOP");


	while( ros::ok() )
	{

		// get the next event from the keyboard
		if(read( kfd_, &c, 1 ) < 0)
		{
			perror("read():");
			exit(-1);
		}

		switch( c )
		{
		// Walking
		case KEYCODE_W:
			vel_x = vel_x + 0.1;
			break;
		case KEYCODE_S:
			vel_x = vel_x - 0.1;
			break;
		case KEYCODE_A:
			vel_omega = vel_omega + 0.5;
			break;
		case KEYCODE_D:
			vel_omega = vel_omega - 0.5;
			break;

		case KEYCODE_SPACE:
			vel_x = vel_omega = 0.0;
			break;

		default:
			break;
		}

		boost::mutex::scoped_lock lock(publish_mutex_);
		if (ros::Time::now() > last_publish_ + ros::Duration(1.0))
		{
			first_publish_ = ros::Time::now();
		}
		last_publish_ = ros::Time::now();
		publish( vel_x, vel_omega );
	}
	return;
}

void RosAriaTeleop::watchdog()
{
	boost::mutex::scoped_lock lock( publish_mutex_ );
	if ((ros::Time::now() > last_publish_ + ros::Duration(0.15)) &&
			(ros::Time::now() > first_publish_ + ros::Duration(0.50)))
		publish(0.0, 0.0);
}

