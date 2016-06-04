#include <stdio.h>      // standard input / output functions
#include <stdlib.h>
#include <string.h>     // string function definitions
#include <unistd.h>     // UNIX standard function definitions
#include <fcntl.h>      // File control definitions
#include <errno.h>      // Error number definitions
#include <termios.h>    // POSIX terminal control definitions
#include <iostream>
#include <mutex>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32MultiArray.h>

#include <snacbot/OpenLids.h>

using namespace std;


class LidService {
public: 
	ros::NodeHandle nh;
	ros::ServiceServer serv;
	ros::Publisher servo_pub;
	int usb;
	mutex mu;

	LidService() {
		servo_pub = nh.advertise<std_msgs::Int32MultiArray>("snacbot/servo", 1000);
		serv = nh.advertiseService("snacbot/lids", &LidService::openLids, this);		
	}

	bool openLids(snacbot::OpenLids::Request &req, snacbot::OpenLids::Response &res) {
		mu.lock();
		ROS_INFO("[openLids] opening...");
		for (auto it = req.snack_ids.begin(); it != req.snack_ids.end(); it++) {
			sendDegree((int) *it, (*it % 2 == 0) ? 90 : 90);
		}
		// Sleep for 5 seconds.
		usleep(1000 * 1000 * 5);
		//for (auto it = req.snack_ids.begin(); it != req.snack_ids.end(); it++) {
		//	sendDegree((int) *it, (*it % 2 == 0) ? 0 : 180);
		//}
		ROS_INFO("[openLids] closing...");
		for (int i = 0; i < 4; i++) {
			sendDegree(i, (i % 2 == 0) ? 180 : 0);
		}
		mu.unlock();
		return true;
	}

	void sendDegree(int servoNum, int degree) {
		std_msgs::Int32MultiArray msg;
		msg.data.push_back(servoNum);
		msg.data.push_back(degree);
		ROS_INFO("[sendDegree] setting servo %d to %d...", servoNum, degree);
		servo_pub.publish(msg);
		ROS_INFO("[sendDegree] sent");
	}
};

int main(int argc, char** argv) {

	ros::init(argc, argv, "lid_service");
	LidService s;
	ros::spin();

	return 0;
}

