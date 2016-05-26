#include <mutex>

#include <ros/ros.h>
#include <snacbot/Order.h>

class Bot {
public:
	Bot() {
		order_sub = nh.subscribe("/snacbot/orders", 1000, &Bot::orderHandler, this);
	}

private:
	ros::NodeHandle nh;
	ros::Subscriber order_sub;
	std::mutex mu;

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
	Bot bot;
	ros::spin();
}
