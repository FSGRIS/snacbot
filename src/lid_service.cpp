#include <stdio.h>      // standard input / output functions
#include <stdlib.h>
#include <string.h>     // string function definitions
#include <unistd.h>     // UNIX standard function definitions
#include <fcntl.h>      // File control definitions
#include <errno.h>      // Error number definitions
#include <termios.h>    // POSIX terminal control definitions
#include <iostream>

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <snacbot/OpenLids.h>

using namespace std;


class LidService {
public: 
	ros::NodeHandle nh;
	ros::ServiceServer serv;
	int USB;

	LidService() {
		USB = open("/dev/ttyACM1", O_RDWR| O_NOCTTY);
		if (USB < 0) {
			cout << "Error, couldn't open USB /dev/ttyACM0, " << errno << endl;
		}
		struct termios tty;
		struct termios tty_old;
		memset(&tty, 0, sizeof(tty));

		if (tcgetattr(USB, &tty) != 0) {
		   cout << "Error " << errno << " from tcgetattr: " << strerror(errno) << std::endl;
		}
		tty_old = tty;
		cfsetospeed(&tty, (speed_t)B9600);
		cfsetispeed (&tty, (speed_t)B9600);
		tty.c_cflag     &=  ~PARENB;            // Make 8n1
		tty.c_cflag     &=  ~CSTOPB;
		tty.c_cflag     &=  ~CSIZE;
		tty.c_cflag     |=  CS8;

		tty.c_cflag     &=  ~CRTSCTS;           // no flow control
		tty.c_cc[VMIN]   =  1;                  // read doesn't block
		tty.c_cc[VTIME]  =  5;                  // 0.5 seconds read timeout
		tty.c_cflag     |=  CREAD | CLOCAL;     // turn on READ & ignore ctrl lines

		/* Make raw */
		cfmakeraw(&tty);

		/* Flush Port, then applies attributes */
		tcflush( USB, TCIFLUSH );
		if ( tcsetattr ( USB, TCSANOW, &tty ) != 0) {
		   std::cout << "Error " << errno << " from tcsetattr" << std::endl;
		}
		serv = nh.advertiseService("snacbot/lids", &LidService::openLids, this);		
	}

	bool openLids(snacbot::OpenLids::Request &req, snacbot::OpenLids::Response &res) {
		for (auto it = req.snack_ids.begin(); it != req.snack_ids.end(); it++) {
			sendDegree((int) *it, (*it % 2 == 0) ? 180 : 0);
		}
		// Sleep for 5 seconds.
		usleep(1000 * 1000 * 5);
		for (auto it = req.snack_ids.begin(); it != req.snack_ids.end(); it++) {
			sendDegree((int) *it, (*it % 2 == 0) ? 0 : 180);
		}
		return true;
	}

	// message format should be "servoNum:degree:"
	int sendDegree(int servoNum, int degree) {//int degree, int USB) {
		ROS_INFO("Setting servo %d to %d...", servoNum, degree);
		int n_written = 0;

		do {
			n_written = write(USB, &servoNum, 1);
			ROS_INFO("n_written: %d", n_written);
		} while (n_written == 0);

		do {
		    n_written = write( USB, &degree, 1 );
			ROS_INFO("n_written: %d", n_written);	    
		} while (n_written == 0);
		ROS_INFO("Sent");
	}
};

int main(int argc, char** argv) {

	ros::init(argc, argv, "lid_service");
	LidService s;
	ros::spin();

	return 0;
}

