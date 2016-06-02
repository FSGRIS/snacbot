#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <mutex>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PointStamped.h>

#include <snacbot/Location.h>
#include <snacbot/GetLocations.h>

using namespace std;

vector<snacbot::Location> parse_locations(string filename) {
	ifstream infile;
	infile.open(filename);
	vector<snacbot::Location> locs;
	long id;
	double x, y;
	while (infile >> id >> x >> y) {
		snacbot::Location l;
		l.id = id;
		l.x = x;
		l.y = y;
		locs.push_back(l);
	}
	return locs;
}

class LocationFileWriter {
public:
	mutex mu;
	ros::NodeHandle nh;
	ros::Subscriber loc_sub;
	ros::Publisher marker_pub;
	ofstream outfile;
	long next_id;
	
	LocationFileWriter(string filename, bool append) {
		marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1000);
		// Wait for subscribers to register before placing markers.
		int n;
		ros::Rate r(10);
		while (ros::ok() && n < 50) {
			// Note: do not run "rostopic echo visualization_marker" or this will not work!
			if (marker_pub.getNumSubscribers() > 0) {
				break;
			}
			n++;
			r.sleep();
		}
		next_id = 0;
		if (append) {
			auto locs = parse_locations(filename);
			// Place markers and set next_id to the max id of the parsed locations.
			for (auto it = locs.begin(); it != locs.end(); it++) {
				placeMarker(*it);
				if (it->id > next_id) {
					next_id = it->id;
				}
			}
		}
		next_id = 1;
		outfile.open(filename, append ? ofstream::app : ofstream::trunc);
		loc_sub = nh.subscribe("clicked_point", 1, &LocationFileWriter::locHandler, this);
	}

	void initLocations(string filename) {
		auto locs = parse_locations(filename);
		// Place markers and set next_id to the max id of the parsed locations.
		for (auto it = locs.begin(); it != locs.end(); it++) {
			placeMarker(*it);
			if (it->id > next_id) {
				next_id = it->id;
			}
		}
		next_id++;
	}

	void placeMarker(snacbot::Location l) {
		visualization_msgs::Marker m;
		m.header.frame_id = "map";
		m.header.stamp = ros::Time::now();
		m.ns = "fast_smart_good";
		m.id = l.id;
		m.frame_locked = true;
		m.type = visualization_msgs::Marker::SPHERE;
		m.action = visualization_msgs::Marker::ADD;
		m.pose.position.x = l.x;
		m.pose.position.y = l.y;
		m.pose.position.z = 1;
		m.pose.orientation.x = 0.0;
		m.pose.orientation.y = 0.0;
		m.pose.orientation.z = 0.0;
		m.pose.orientation.w = 1.0;
		m.scale.x = 0.2;
		m.scale.y = 0.2;
		m.scale.z = 0.2;
		m.color.a = 1.0;
		m.color.r = 0.8;
		m.color.g = 0.2;
		m.color.b = 0.1;
		m.text = to_string(l.id);
		marker_pub.publish(m);
		ROS_INFO("published loc %ld at (%.2f, %.2f)", l.id, l.x, l.y);
		ROS_INFO("marker info %d", marker_pub.getNumSubscribers());
	}

	void locHandler(const geometry_msgs::PointStamped msg) {
		ROS_INFO("[location service] new loc");
		mu.lock();
		snacbot::Location l;
		l.id = next_id;
		l.x = msg.point.x;
		l.y = msg.point.y;
		ROS_INFO("new location: %ld at (%.2f, %.2f)", l.id, l.x, l.y);
		next_id++;
		outfile << l.id << "\t" << l.x << "\t" << l.y << endl;
		placeMarker(l);
		mu.unlock();
	}
};

class LocationService {
public:
	ros::NodeHandle nh;
	ros::ServiceServer serv;
	vector<snacbot::Location> locs;

	LocationService(string filename) {
		serv = nh.advertiseService("snacbot/locations", &LocationService::getLocations, this);
		locs = parse_locations(filename);
	}

	bool getLocations(snacbot::GetLocations::Request &req, snacbot::GetLocations::Response &res) {
		res.locs = locs;
		return true;
	}
};

void usage() {
	cout << "Usage:" << endl;
	cout << "location_service [-wa] [output_file]" << endl;
	cout << "Options:" << endl;
	cout << "\t-w write new file" << endl;
	cout << "\t-a append to existing file" << endl;
	exit(1);
}

int main(int argc, char **argv) {
	if (argc < 2) {
		usage();
	}
	ros::init(argc, argv, "location_service");
	bool overwrite = strcmp(argv[1], "-w") == 0;
	bool append = strcmp(argv[1], "-a") == 0;
	if (overwrite || append) {
		if (argc != 3) {
			// No output file provided.
			usage();
		}
		LocationFileWriter w(argv[2], append);
		ros::spin();
	} else {
		LocationService s(argv[1]);
		ros::spin();
	}
}
