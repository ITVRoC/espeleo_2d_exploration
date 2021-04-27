#include <iostream>
#include <vector>
#include <chrono>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <tf/transform_listener.h>
#include "tf/transform_broadcaster.h"
#include "tf/message_filter.h"
#include "espeleo_2d_exploration/Frontier.h"
#include "espeleo_2d_exploration/FrontierArray.h"
#include "tf/transform_datatypes.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "actionlib/client/simple_action_client.h"
#include <cstdlib> // Needed for rand()
#include <ctime> // Needed to seed random number generator with a time value

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

typedef struct
{
    float x;
    float y;
} Point2D;

class EspeleoExploration
{
private:
	sensor_msgs::PointCloud cluster_cloud;
    ros::NodeHandle nh_;
    ros::Publisher cmd_vel_pub_;
    ros::Publisher frontier_centers_pub;
    ros::Subscriber frontier_sub;
    tf::TransformListener *tfListener;
    MoveBaseClient ac;
    bool exploration_finished;
    std::string exploration_mode;
    std::chrono::time_point<std::chrono::high_resolution_clock> t_ini;

public:
    EspeleoExploration(ros::NodeHandle &nh, tf::TransformListener &list) : ac("/move_base", true)
    {
        srand(time(NULL));
        nh_ = nh;
        tfListener = &list;
        nh_.param<std::string>("mode", exploration_mode, "largest");
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
        frontier_centers_pub = nh_.advertise<sensor_msgs::PointCloud>("/cluster_centers", 1);
        frontier_sub = nh_.subscribe("/frontiers", 1, &EspeleoExploration::frontierCallback, this);
        cluster_cloud.header.frame_id = "map";
        exploration_finished = false;
        t_ini = std::chrono::high_resolution_clock::now();
        while(!ac.waitForServer(ros::Duration(5.0)))
        {
            ROS_INFO("Waiting for the move_base action server to come up");            
        } 
    };

    float getDistance(float x1, float x2, float y1, float y2)
	{
		return sqrt(pow((x1-x2), 2.) + pow((y1 - y2), 2.));
	};

    
    espeleo_2d_exploration::FrontierArray clusterFrontiers(const espeleo_2d_exploration::FrontierArray &frontiers_in)
    {
        espeleo_2d_exploration::FrontierArray frontiers;
        for(int i = 0; i < frontiers_in.frontiers.size(); i++)
        {
            frontiers.frontiers.push_back(frontiers_in.frontiers[i]);
        }

        espeleo_2d_exploration::FrontierArray frontiersCluster;
        if(frontiers.frontiers.size() == 0)
        {
            return frontiers;
        }
        Point2D ref;
        Point2D ref_ini;
        
        bool create_new_frontier = true;
        espeleo_2d_exploration::Frontier currentFrontier;
        ref.x = frontiers.frontiers[0].center.x;
        ref.y = frontiers.frontiers[0].center.y;
        
        float distance;
        while(frontiers.frontiers.size() > 0)
        {
            if(create_new_frontier)
            {
                ref.x = frontiers.frontiers[0].center.x;
                ref.y = frontiers.frontiers[0].center.y;
                ref_ini.x = ref.x;
                ref_ini.y = ref.y;
                frontiers.frontiers.erase(frontiers.frontiers.begin());
                create_new_frontier = false;
            }
            for(int i = 0; i < frontiers.frontiers.size(); i++)
            {
                distance = getDistance(ref.x, frontiers.frontiers[i].center.x, ref.y, frontiers.frontiers[i].center.y);
                if(distance < .5)
                {
                    geometry_msgs::Point32 point;
                    point.x = frontiers.frontiers[i].center.x;
                    point.y = frontiers.frontiers[i].center.y;
                    currentFrontier.points.push_back(point);
                    ref.x = frontiers.frontiers[i].center.x;
                    ref.y = frontiers.frontiers[i].center.y;
                    frontiers.frontiers.erase(frontiers.frontiers.begin()+i);
                    break;
                }

                if(i == frontiers.frontiers.size()-1)
                {
                    ///do 2nd run to cluster possible remaining points of the frontier
                    ref.x = ref_ini.x;
                    ref.y = ref_ini.y;
                    for(int j = 0; j < frontiers.frontiers.size(); j++)
                    {
                        distance = getDistance(ref.x, frontiers.frontiers[j].center.x, ref.y, frontiers.frontiers[j].center.y);
                        if(distance < .5)
                        {
                            geometry_msgs::Point32 point;
                            point.x = frontiers.frontiers[j].center.x;
                            point.y = frontiers.frontiers[j].center.y;
                            currentFrontier.points.push_back(point);
                            ref.x = frontiers.frontiers[j].center.x;
                            ref.y = frontiers.frontiers[j].center.y;
                            frontiers.frontiers.erase(frontiers.frontiers.begin()+j);
                        }
                    }
                    ///
                    if(currentFrontier.points.size() > 4)
                    {
                        frontiersCluster.frontiers.push_back(currentFrontier);
                    }
                    create_new_frontier = true;
                    currentFrontier.points.clear();
                }
            }
        }
        cluster_cloud.points.resize(frontiersCluster.frontiers.size());
        for(int i = 0; i < frontiersCluster.frontiers.size(); i++)
        {
            float avg_x = 0;
            float avg_y = 0;
            if(frontiersCluster.frontiers[i].points.size() == 0)
            {
                continue;
            }
            for(int j = 0; j < frontiersCluster.frontiers[i].points.size(); j++)
            {
                avg_x += frontiersCluster.frontiers[i].points[j].x;
                avg_y += frontiersCluster.frontiers[i].points[j].y;
            }
            avg_x = avg_x/frontiersCluster.frontiers[i].points.size();
            avg_y = avg_y/frontiersCluster.frontiers[i].points.size();
            cluster_cloud.points[i].x = avg_x;
            cluster_cloud.points[i].y = avg_y;
            frontiersCluster.frontiers[i].center.x = avg_x;
            frontiersCluster.frontiers[i].center.y = avg_y;
        }
        frontier_centers_pub.publish(cluster_cloud);
        return frontiersCluster;
    };

    void frontierCallback(const espeleo_2d_exploration::FrontierArray &frontiers)
	{
        
        espeleo_2d_exploration::FrontierArray frontierCluster = clusterFrontiers(frontiers);

        if(frontierCluster.frontiers.size() == 0)
        {
            auto t_end = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>( t_end - t_ini ).count();
            exploration_finished = true;
            std_msgs::Float32::ConstPtr sharedDist;
            std_msgs::Float32 dist_travelled;
            sharedDist = ros::topic::waitForMessage<std_msgs::Float32>("/distance_travelled");
            if(sharedDist != NULL)
            {
                dist_travelled = *sharedDist;
            }
            float dist = dist_travelled.data;
            ROS_INFO("Received 0 frontiers, exploration is done after %f microseconds. Distance travelled: %f meters", duration, dist);
            ros::shutdown();
        }

        // for (int i = 0; i < frontierCluster.frontiers.size(); i++)
        // {
        //     std::cout << "Frontier " << i << " has " << frontierCluster.frontiers[i].points.size() << " points." << std::endl;
        // }
        
		//ROS_INFO("Frontier callback!");
		tf::StampedTransform transform;
		tfListener->waitForTransform("map", "base_link", ros::Time(0), ros::Duration(3.0));
		tfListener->lookupTransform("map", "base_link", ros::Time(0), transform);
		//		
		int frontier_i = 0;
        int second_frontier = 0;
        int largest_frontier = 0;
        int largest_frontier_size = 0;
        int target_frontier;    
		float closest_frontier_distance = 100000, distance = 0;
        std::vector<float> distances;
        //std::vector<std::pair<float, float>> points;
		ROS_INFO("Navigation got %d frontiers", frontierCluster.frontiers.size());
		if(frontierCluster.frontiers.size() == 0)
			return;
		//
		for(int i = 0; i < frontierCluster.frontiers.size(); i++) {
			distance = getDistance(frontierCluster.frontiers[i].center.x, transform.getOrigin().x(), frontierCluster.frontiers[i].center.y, transform.getOrigin().y());

			if(distance > .8 && distance <= closest_frontier_distance) {
				closest_frontier_distance = distance;
                second_frontier = frontier_i;
				frontier_i = i;
			}
            if(distance > .8 && frontierCluster.frontiers[i].points.size() > largest_frontier_size)
            {
                largest_frontier = i;
                largest_frontier_size = frontierCluster.frontiers[i].points.size();
            }
		}

        if(exploration_mode == "largest")
        {
            target_frontier = largest_frontier;
        }
        else if(exploration_mode == "closest")
        {
            target_frontier = frontier_i;
        }
        

		ROS_INFO("Closest distance: %f", closest_frontier_distance);
        ROS_INFO("Largest frontier has %i points", largest_frontier_size);
        ROS_INFO("Frontier index: %i", target_frontier);

		move_base_msgs::MoveBaseGoal goal;
		goal.target_pose.header.frame_id = "map";
		goal.target_pose.header.stamp = ros::Time::now();
        // ROS_INFO("Frontier index: %d", frontier_i);			
        goal.target_pose.pose.position.x = frontierCluster.frontiers[target_frontier].center.x;
        goal.target_pose.pose.position.y = frontierCluster.frontiers[target_frontier].center.y;
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0);
        goal.target_pose.pose.orientation = odom_quat;
        ROS_INFO("Navigating from x: %f y: %f to: x: %f y: %f", transform.getOrigin().x(), goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);

        ac.sendGoal(goal);
        ac.waitForResult(ros::Duration(20.0));
        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("The base moved to %f,%f", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);
        }
	};

    void spin()
    {
        ros::Rate rate(10);
        while(ros::ok())
        {
            ros::spinOnce();
            rate.sleep();
        }
    };
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "espeleo_2d_exploration");
    ros::NodeHandle nh("~");
    tf::TransformListener listener;
    EspeleoExploration explorer(nh, listener);
    std::cout << "Spinning..." << std::endl;
    explorer.spin();
    return 0;
}