#include <stdlib.h>
#include <mutex>

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <snacbot/Order.h>
#include <snacbot/Location.h>
#include <snacbot/GetLocations.h>
#include <snacbot/OpenLids.h>

using namespace std;

typedef struct {
	double x;
	double y;
} Point;

class Snacbot {
public:
	ros::NodeHandle nh;
	ros::Subscriber order_sub;
	ros::ServiceClient lid_client;
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
		lid_client = nh.serviceClient<snacbot::OpenLids>("snacbot/lids");
		ROS_INFO("end of constructor");
	}

	void orderHandler(const snacbot::Order msg) {
		mu.lock();
		ROS_INFO("new order!");
		ROS_INFO("location_id: %ld", msg.location_id);
		for (auto it = msg.snack_ids.begin(); it != msg.snack_ids.end(); it++) {
			ROS_INFO("\tsnack_id: %ld", *it);
		}
		openLids(msg.snack_ids);
		/*auto it = loc_map.find(msg.location_id);
		if (it == loc_map.end()) {
			ROS_INFO("error: location %ld not found", msg.location_id);
			return;
		}
		Point p = it->second;
		actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
		while (!ac.waitForServer(ros::Duration(0.5))) {
			ROS_INFO("waiting for move_base action server to come up");
		}
		move_base_msgs::MoveBaseGoal goal;
		goal.target_pose.header.frame_id = "map";
		goal.target_pose.pose.position.x = p.x;
		goal.target_pose.pose.position.y = p.y;
		goal.target_pose.pose.orientation.w = 1.0;
		ROS_INFO("sending goal to (%f, %f)", p.x, p.y);
		ac.sendGoal(goal);
		ac.waitForResult();
		if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
			ROS_INFO("move succeeded");
			openLids(msg.snack_ids);
		} else {
			ROS_INFO("move failed");
			ROS_INFO("current state: %s", ac.getState().toString().c_str());
		}*/
		mu.unlock();
	}

	void openLids(vector<long> snack_ids) {
		snacbot::OpenLids srv;
		srv.request.snack_ids = snack_ids;
		if (lid_client.call(srv)) {
			// Success!
			ROS_INFO("lids successfully set");
		} else {
			// Failure.
			ROS_INFO("error setting lids");
		}
	}
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "snacbot");
	Snacbot bot;
	ros::spin();
}
