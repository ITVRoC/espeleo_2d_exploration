#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include "tf/transform_broadcaster.h"
#include "tf/message_filter.h"
#include "tf/transform_datatypes.h"

typedef struct
{
    float x;
    float y;
} Point2D;

class MeasureDistance
{
    private:
    ros::NodeHandle nh_;
    ros::Subscriber odom_sub;
    ros::Publisher distance_pub;
    std_msgs::Float32 distance_msg;
    tf::TransformListener *tfListener;
    float distance_travelled;
    Point2D previous_pos;
    Point2D current_pos;
    
    public:
    MeasureDistance(ros::NodeHandle &nh, tf::TransformListener &list)
    {
        nh_ = nh;
        tfListener = &list;
        previous_pos.x = 0;
        previous_pos.y = 0;
        odom_sub = nh_.subscribe("/odom", 1, &MeasureDistance::odomCallback, this);
        distance_pub = nh_.advertise<std_msgs::Float32>("/distance_travelled", 1);
    };

    float getDistance(float x1, float x2, float y1, float y2)
	{
		return sqrt(pow((x1-x2), 2.) + pow((y1 - y2), 2.));
	};

    void odomCallback(const nav_msgs::Odometry &odom)
    {
        tf::StampedTransform transform;
		tfListener->waitForTransform("map", "base_link", ros::Time(0), ros::Duration(3.0));
		tfListener->lookupTransform("map", "base_link", ros::Time(0), transform);

        current_pos.x = transform.getOrigin().x();
        current_pos.y = transform.getOrigin().y();
        float distance = getDistance(current_pos.x, previous_pos.x, current_pos.y, previous_pos.y);
        distance_travelled += distance;

        previous_pos.x = current_pos.x;
        previous_pos.y = current_pos.y;

        std::cout << "Distance traveled: " << distance_travelled << std::endl;

        distance_msg.data = distance_travelled;
        distance_pub.publish(distance_msg);
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
    ros::init(argc, argv, "distance_measurer");
    ros::NodeHandle nh;
    tf::TransformListener listener;
    MeasureDistance measurer(nh, listener);
    measurer.spin();
    return 0;
}