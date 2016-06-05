#include <stdlib.h>
#include <mutex>
#include <atomic>

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>

#include <snacbot/Order.h>
#include <snacbot/Location.h>
#include <snacbot/GetLocations.h>
#include <snacbot/SetLids.h>

using namespace std;

typedef struct {
	double x;
	double y;
} Point;

class Snacbot {
public:
	ros::NodeHandle nh;
	Point home;

	atomic<bool> handlingOrder;
	ros::Subscriber order_sub;
	ros::ServiceClient lid_client;
	map<long, Point> loc_map;
	ros::Subscriber done_sub;
	
	mutex waitMu;
	int numWaiting;

	bool servingSnacks;

	Snacbot() {
		tf::TransformListener tl(nh);
		tl.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(5));
		tf::StampedTransform t;
		tl.lookupTransform("map", "base_link", ros::Time(0), t);
		home.x = t.getOrigin().x();
		home.y = t.getOrigin().y();
		order_sub = nh.subscribe("snacbot/orders", 1000, &Snacbot::orderHandler, this);
		done_sub = nh.subscribe("snacbot/done", 1, &Snacbot::doneHandler, this);
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
		lid_client = nh.serviceClient<snacbot::SetLids>("snacbot/lids");
		servingSnacks = false;
		handlingOrder = false;
		ROS_INFO("snacbot started");
	}

	bool move(Point p) {
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
			return true;
		} else {
			ROS_INFO("move failed");
			ROS_INFO("current state: %s", ac.getState().toString().c_str());
			return false;
		}
	}

	void orderHandler(const snacbot::Order msg) {
		ROS_INFO("new order (waiting)");
		waitMu.lock();
		numWaiting++;
		waitMu.unlock();

		ros::Rate r(1);
		while (handlingOrder) {
			ros::spinOnce();
			r.sleep();
		}
		handlingOrder = true;

		ROS_INFO("serving order");
		ROS_INFO("location_id: %ld", msg.location_id);
		for (auto it = msg.snack_ids.begin(); it != msg.snack_ids.end(); it++) {
			ROS_INFO("\tsnack_id: %ld", *it);
		}
		auto it = loc_map.find(msg.location_id);
		if (it == loc_map.end()) {
			ROS_INFO("error: location %ld not found", msg.location_id);
			return;
		}
		Point dest = it->second;
		while (!move(dest)) {
			ROS_INFO("move failed");
		}
		ROS_INFO("opening lids");
		servingSnacks = true;
		openLids(msg.snack_ids);
	}

	void doneHandler(const std_msgs::String &msg) {
		ROS_INFO("done clicked");
		if (!servingSnacks) {
			return;
		}
		closeLids();
		servingSnacks = false;

		waitMu.lock();
		numWaiting--;
		if (numWaiting == 0) {
			move(home);
		}
		waitMu.unlock();
	
		handlingOrder = false;
	}

	void closeLids() {
		snacbot::SetLids srv;
		srv.request.open = false;
		if (lid_client.call(srv)) {
			ROS_INFO("lids successfully closed");
		} else {
			ROS_INFO("error closing lids");
		}
	}

	void openLids(vector<long> snack_ids) {
		snacbot::SetLids srv;
		srv.request.open = true;
		srv.request.snack_ids = snack_ids;
		if (lid_client.call(srv)) {
			ROS_INFO("lids successfully opened");
		} else {
			ROS_INFO("error opening lids");
		}
	}
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "snacbot");
	Snacbot bot;
	ros::spin();
}
