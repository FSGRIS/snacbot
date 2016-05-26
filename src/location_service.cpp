#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <mutex>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PointStamped.h>

#include <snacbot/Location.h>

using namespace std;

vector<snacbot::Location> parse_locations(string filename) {
	ifstream infile;
	infile.open(filename);
	vector<snacbot::Location> locs;
	long id, x, y;
	while (infile >> id >> x >> y) {
		snacbot::Location l;
		l.id = id;
		l.x = x;
		l.y = y;
		locs.push_back(l);
	}
	return locs;
}

class LocFileWriter {
public:
	mutex mu;
	ros::NodeHandle nh;
	ros::Subscriber point_sub;
	ros::Publisher marker_pub;
	fstream outfile;
	vector<snacbot::Location> locs;
	long next_id;
	
	LocFileWriter(string filename, bool append) {
		ROS_INFO("in writer");
		if (append) {
			locs = parse_locations(filename);
			// Place markers and set next_id to the max id of the parsed locations.
			for (auto it = locs.begin(); it != locs.end(); it++) {
				placeMarker(*it);
				if (it->id > next_id) {
					next_id = it->id;
				}
			}
		}
		next_id++;
		outfile.open(filename, append ? ofstream::app : ofstream::trunc);
		point_sub = nh.subscribe("/clicked_point", 1, &LocFileWriter::pointHandler, this);
		marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
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
		m.scale.x = 0.1;
		m.scale.y = 0.1;
		m.scale.z = 0.1;
		m.color.a = 1.0; // Don't forget to set the alpha!
		m.color.r = 0.8;
		m.color.g = 0.2;
		m.color.b = 0.1;
		m.text = to_string(l.id);
		marker_pub.publish(m);
	}

	void pointHandler(const geometry_msgs::PointStamped msg) {
		mu.lock();
		ROS_INFO("point placed");
		snacbot::Location l;
		l.x = msg.point.x;
		l.y = msg.point.y;
		l.id = next_id;
		ROS_INFO("new location: %ld at (%ld, %ld)", l.id, l.x, l.y);
		next_id++;
		locs.push_back(l);
		placeMarker(l);
		mu.unlock();
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
		LocFileWriter w(argv[2], append);
	} else {
		ROS_INFO("?");
	}
	ros::spin();
}
