#include <stdlib.h>
#include <mutex>

#include <ros/ros.h>
#include <snacbot/Order.h>
#include <snacbot/Location.h>
#include <snacbot/GetLocations.h>

using namespace std;

typedef struct {
	long x;
	long y;
} Point;

class Snacbot {
public:
	ros::NodeHandle nh;
	ros::Subscriber order_sub;
	mutex mu;
	map<long, Point> loc_map;

	Snacbot() {
		order_sub = nh.subscribe("/snacbot/orders", 1000, &Snacbot::orderHandler, this);
		ros::ServiceClient cli = nh.serviceClient<snacbot::GetLocations>("snacbot/locations");
		snacbot::GetLocations srv;
		if (cli.call(srv)) {
			auto locs = srv.response.locs;
			for (auto it = locs.begin(); it != locs.end(); it++) {
				Point p;
				p.x = it->x;
				p.y = it->y;
				loc_map[it->id] = p;
			}
		}
	}

	void orderHandler(const snacbot::Order msg) {
		mu.lock();
		ROS_INFO("new order!");
		ROS_INFO("location_id: %d", (int) msg.location_id);
		for (auto it = msg.snacks.begin(); it != msg.snacks.end(); it++) {
			ROS_INFO("\tsnack_id: %d, quantity: %d", (int) it->id, (int) it->quantity);
		}
		ROS_INFO("----------");
		mu.unlock();
	}
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "snacbot");
	Snacbot bot;
	ros::spin();
}
