#include <iostream>
#include <vector>
#include <chrono>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <tf/transform_listener.h>
#include "tf/transform_broadcaster.h"
#include "tf/message_filter.h"
#include "espeleo_2d_exploration/Frontier.h"
#include "espeleo_2d_exploration/FrontierArray.h"
#include "espeleo_control/Path.h"
#include "espeleo_control/NavigatePathAction.h"
#include "tf/transform_datatypes.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "actionlib/client/simple_action_client.h"
#include <cstdlib> // Needed for rand()
#include <ctime> // Needed to seed random number generator with a time value

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
//typedef actionlib::SimpleActionClient<espeleo_control::NavigatePathAction> EspeleoControlClient;

typedef struct
{
    float x;
    float y;
} Point2D;

class EspeleoExploration
{
private:
	sensor_msgs::PointCloud cluster_cloud;
    geometry_msgs::PointStamped goal_point_pub;
    nav_msgs::Odometry odom;
    Point2D position;
    ros::NodeHandle nh_;
    ros::Publisher cmd_vel_pub_;
    ros::Publisher frontier_centers_pub;
    ros::Publisher goal_pub;
    ros::Subscriber frontier_sub;
    ros::Subscriber path_sub;
    ros::Subscriber odom_sub;
    tf::TransformListener *tfListener;
    MoveBaseClient ac;
    std::chrono::time_point<std::chrono::high_resolution_clock> t_ini;
    bool exploration_finished;
    int num_commands;

    //config params
    int min_frontier_size;
    float spin_for;
    std::string exploration_mode;
    

public:
    EspeleoExploration(ros::NodeHandle &nh, tf::TransformListener &list) : ac("/move_base", true)
    {
        srand(time(NULL));
        nh_ = nh;
        tfListener = &list;
        nh_.param<std::string>("mode", exploration_mode, "largest");
        nh_.param<int>("min_frontier_size", min_frontier_size, 4);
        nh_.param<float>("spin_for", spin_for, 5.0);
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
        frontier_centers_pub = nh_.advertise<sensor_msgs::PointCloud>("/cluster_centers", 1);
        goal_pub = nh_.advertise<geometry_msgs::PointStamped>("/goal_point", 1);
        frontier_sub = nh_.subscribe("/frontiers", 1, &EspeleoExploration::frontierCallback, this);
        //path_sub = nh_.subscribe("/final_path", 1, &EspeleoExploration::pathCallback, this);
        odom_sub = nh_.subscribe("/odom", 1, &EspeleoExploration::odomCallback, this);
        cluster_cloud.header.frame_id = "map";
        exploration_finished = false;
        t_ini = std::chrono::high_resolution_clock::now();
        while(!ac.waitForServer(ros::Duration(5.0)))
        {
            ROS_INFO("Waiting for the espeleo_control action server to come up");            
        }
        num_commands = 0;
        ROS_INFO("mode: %s, min_frontier_size: %i, spin_for: %f", exploration_mode, min_frontier_size, spin_for);
        spinFor(spin_for);
    };

    float getDistance(float x1, float x2, float y1, float y2)
	{
		return sqrt(pow((x1-x2), 2.) + pow((y1 - y2), 2.));
	};

    void spinFor(float time)
    {
        geometry_msgs::Twist vel_msg;
        vel_msg.angular.z = 0.2;
        ros::Time beginTime = ros::Time::now();
        ros::Duration secondsIWantToSendMessagesFor = ros::Duration(spin_for); 
        ros::Time endTime = beginTime + secondsIWantToSendMessagesFor;
        while(ros::Time::now() < endTime )
        {
        cmd_vel_pub_.publish(vel_msg);
        //ros::Duration(0.1).sleep();
        }
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

    void odomCallback(const nav_msgs::Odometry &odom)
    {
        //position.x = odom.pose.pose.position.x;
        //position.y = odom.pose.pose.position.y;
        //ROS_INFO("Robot position: (x = %f; y = %f)", position.x, position.y);
    }

    void frontierCallback(const espeleo_2d_exploration::FrontierArray &frontiers)
	{
        
        espeleo_2d_exploration::FrontierArray frontierCluster = clusterFrontiers(frontiers);
        ROS_INFO("Frontier points: %i", frontiers.frontiers.size());
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
		tfListener->waitForTransform("map", "odom", ros::Time(0), ros::Duration(3.0));
		tfListener->lookupTransform("map", "odom", ros::Time(0), transform);
		//
        position.x = transform.getOrigin().x();
        position.y = transform.getOrigin().y();
        std::cout << "position.x: " << position.x << " - position.y: " << position.y << std::endl;
        std::cout << "got position" << std::endl;
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
			distance = getDistance(frontierCluster.frontiers[i].center.x, position.x, frontierCluster.frontiers[i].center.y, position.y);
            std::cout << "Calc dist: " << distance << std::endl;
			if(distance > .1 && distance <= closest_frontier_distance) {
				closest_frontier_distance = distance;
                second_frontier = frontier_i;
				frontier_i = i;
                std::cout << "Chosen dist: " << distance << std::endl;
			}
            if(distance > .1 && frontierCluster.frontiers[i].points.size() > largest_frontier_size)
            {
                largest_frontier = i;
                largest_frontier_size = frontierCluster.frontiers[i].points.size();
            }
		}

        if(frontierCluster.frontiers.size() == 1 && second_frontier == 0)
        {
            second_frontier = 1;
            std::cout << "assigning second frontier" << std::endl;
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

        espeleo_control::NavigatePathGoal goal_path;
        goal_path.path.header.frame_id = "map";
        geometry_msgs::Point32 goal_point;
        goal_point = getClosestFrontierPoint(frontierCluster.frontiers[target_frontier], position.x, position.y);
        move_base_msgs::MoveBaseGoal goal;
		goal.target_pose.header.frame_id = "map";
		goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = goal_point.x;
        goal.target_pose.pose.position.y = goal_point.y;
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0);
        goal.target_pose.pose.orientation = odom_quat;
        //goal_point.x = frontierCluster.frontiers[target_frontier].center.x;
        //goal_point.y = frontierCluster.frontiers[target_frontier].center.y;
        //goal_point.z = 0;

        //goal_path.path.path.points.push_back(goal_point);
        //std::cout << "GOAL PATH POINTS: " << goal_path.path.path.points.size() << std::endl;
        goal_point_pub.header.frame_id = "map";
        goal_point_pub.point.x = goal_point.x;
        goal_point_pub.point.y = goal_point.y;
        goal_point_pub.point.z = 0;
        goal_pub.publish(goal_point_pub);
        num_commands++;
        ROS_INFO("Navigating from x: %f y: %f to: x: %f y: %f", position.x, position.y, goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);
        ROS_INFO("Number of commands: %i", num_commands);
        ac.sendGoal(goal);
        bool success = ac.waitForResult(ros::Duration(10.0));
        

        if(success)
        {
            ROS_INFO("The base moved to %f,%f", goal_point.x, goal_point.y);
        }
        else
        {
            ROS_INFO("Could not reach target goal, moving to next frontier...");
            goal_point = getClosestFrontierPoint(frontierCluster.frontiers[second_frontier], position.x, position.y);
            goal.target_pose.pose.position.x = goal_point.x;
            goal.target_pose.pose.position.y = goal_point.y;
            ac.sendGoal(goal);
        }
        
	};

    void pathCallback(const nav_msgs::Path path)
    {
        espeleo_control::NavigatePathGoal goalPath;
        goalPath.path.header.frame_id = "map";
        goalPath.path.closed_path_flag = false;
        //goalPath.path.insert_n_points = 1;
        goalPath.path.filter_path_n_average = 0;
        for(int i = 0; i < path.poses.size(); i++)
        {
            geometry_msgs::Point32 pathPoint;
            pathPoint.x = path.poses[i].pose.position.x;
            pathPoint.y = path.poses[i].pose.position.y;
            pathPoint.z = 0;
            goalPath.path.path.points.push_back(pathPoint);
        }
        //ec.sendGoal(goalPath);
        //num_commands = num_commands + 1;
        //ROS_INFO("Sending action: %i", num_commands);
    };

    geometry_msgs::Point32 getClosestFrontierPoint(espeleo_2d_exploration::Frontier targetFrontier, float robotPoseX, float robotPoseY)
    {
        int index;
        float d, x, y;
        float closest_distance = 100000;
        geometry_msgs::Point32 closest_point;
        for(int i = 0; i < targetFrontier.points.size(); i++)
        {
            d = getDistance(targetFrontier.points[i].x, robotPoseX, targetFrontier.points[i].y, robotPoseY);
            if(d < closest_distance) {
				closest_distance = d;
                closest_point = targetFrontier.points[i];
			}
        }
        return closest_point;
    };

    void spin()
    {
        ros::Rate rate(10);
        while(ros::ok())
        {
            goal_pub.publish(goal_point_pub);
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