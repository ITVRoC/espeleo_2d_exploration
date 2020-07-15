#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "nav_msgs/OccupancyGrid.h"

void callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
  ROS_INFO("I heard: [%d]",msg->info.width);
  std::cout << "width:" << msg->info.width << "height: " << msg->info.resolution << std::endl;
  for(int i = 0; i < msg->data.size(); i++)
  {
    if(msg->data[i] != -1)
    {
      std::cout << static_cast<int16_t>(msg->data[i]) << std::endl;
    }
  }
  //map_pub.publish(msg->info.width);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "map_info");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/map", 1000, callback);
  //ros::Publisher map_pub = n.advertise<int>("mapconverted", 1000);
  ros::Rate loop_rate(10);

 int count = 0;
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  ros::spin();
}