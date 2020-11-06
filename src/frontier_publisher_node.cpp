#include "ros/ros.h"
#include "ros/console.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
// For transform support
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "tf/transform_datatypes.h"
#include "sensor_msgs/PointCloud.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "actionlib/client/simple_action_client.h"
#include <cstdlib> // Needed for rand()
#include <ctime> // Needed to seed random number generator with a time value
#include "wavefront_frontier_detection.cpp"
#include "espeleo_2d_exploration/Frontier.h"
#include "espeleo_2d_exploration/FrontierArray.h"

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
class Exploration2D {

public:
	// Construst a new RandomWalk object and hook up this ROS node
	// to the simulated robot's velocity control and laser topics
	Exploration2D(ros::NodeHandle& nh) {
		// Initialize random time generator
		srand(time(NULL));
		// Advertise a new publisher for the simulated robot's velocity command topic
		// (the second argument indicates that if multiple command messages are in
		//  the queue to be sent, only the last command will be sent)
		frontier_publisher = nh.advertise<sensor_msgs::PointCloud>("/frontier_cloud", 1);
		frontier_center_publisher = nh.advertise<sensor_msgs::PointCloud>("/frontier_centers", 1);
		frontier_array_publisher = nh.advertise<espeleo_2d_exploration::FrontierArray>("/frontiers", 1);
		// Subscribe to the simulated robot's laser scan topic and tell ROS to call
		// this->commandCallback() whenever a new message is published on that topic
		//laserSub = nh.subscribe("base_scan", 1, &TurtlebotExploration::commandCallback,this);
		mapSub = nh.subscribe("/map", 1, &Exploration2D::mapCallback, this);
		//
		frontier_cloud.header.frame_id = "map";
		frontier_centers.header.frame_id = "map";
	}
	;

	void mapCallback( const nav_msgs::OccupancyGrid& map )
	{
		//
		float resolution = map.info.resolution;
		float map_x = map.info.origin.position.x / resolution;
		float map_y = map.info.origin.position.y / resolution;
		float x = 0. - map_x;
		float y = 0. - map_y;
		vector<vector<int> > frontiers = wfd(map, map.info.height, map.info.width, x + (y * map.info.width));
		int num_points = 0;
		for(int i = 0; i < frontiers.size(); i++) {
			for(int j = 0; j < frontiers[i].size(); j++) {
				num_points++;
			}
		}
		//
		frontier_cloud.points.resize(num_points);
		frontier_centers.points.resize(frontiers.size());
		int pointI = 0;
		frontier_array.frontiers.clear();
		std::cout << "frontiers size:" << frontiers.size() << std::endl;
		for(int i = 0; i < frontiers.size(); i++) {
			espeleo_2d_exploration::Frontier current_frontier;
			current_frontier.points.resize(frontiers[i].size());
			float x_avg = 0;
			float y_avg = 0;
			for(int j = 0; j < frontiers[i].size(); j++) {
				frontier_cloud.points[pointI].x = ((frontiers[i][j] % map.info.width) + map_x) * resolution;
				frontier_cloud.points[pointI].y = ((frontiers[i][j] / map.info.width) + map_y) * resolution;
				frontier_cloud.points[pointI].z = 0;
				current_frontier.points[j].x = ((frontiers[i][j] % map.info.width) + map_x) * resolution;
				current_frontier.points[j].y = ((frontiers[i][j] / map.info.width) + map_y) * resolution;
				x_avg += current_frontier.points[j].x;
				y_avg += current_frontier.points[j].y;
				current_frontier.points[j].z = 0;
				pointI++;
			}
			x_avg = x_avg/frontiers[i].size();
			y_avg = y_avg/frontiers[i].size();
			frontier_centers.points[i].x = x_avg;
			frontier_centers.points[i].y = y_avg;
			current_frontier.center.x = x_avg;
			current_frontier.center.y = y_avg;
			current_frontier.center.z = 0;
			frontier_array.frontiers.push_back(current_frontier);
		}
		//
		std::cout << "No. of frontiers: " << frontier_array.frontiers.size();
		frontier_publisher.publish(frontier_cloud);
		frontier_array_publisher.publish(frontier_array);
		frontier_center_publisher.publish(frontier_centers);
		//ROS_INFO("published cloud! Size: %d.", num_points);
	};

	// Main FSM loop for ensuring that ROS messages are
	// processed in a timely manner, and also for sending
	// velocity controls to the simulated robot based on the FSM state
	void spin() {
		ros::Rate rate(1); // Specify the FSM loop rate in Hz
		while (ros::ok()) { // Keep spinning loop until user presses Ctrl+C
			ros::spinOnce(); // Need to call this function often to allow ROS to process incoming messages
			frontier_publisher.publish(frontier_cloud);
			frontier_array_publisher.publish(frontier_array);
			rate.sleep(); // Sleep for the rest of the cycle, to enforce the FSM loop rate
		}
	};

protected:
	sensor_msgs::PointCloud frontier_cloud;
	sensor_msgs::PointCloud frontier_centers;
	espeleo_2d_exploration::FrontierArray frontier_array;
	ros::Publisher frontier_publisher;
	ros::Publisher frontier_array_publisher;
	ros::Publisher frontier_center_publisher;
	ros::Subscriber mapSub;
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "espeleo_2d_exploration_node"); // Initiate new ROS node named "random_walk"
	ros::NodeHandle n;
	int a = teste();
	Exploration2D walker(n);
	walker.spin(); // Execute FSM loop
	return 0;
}
;
