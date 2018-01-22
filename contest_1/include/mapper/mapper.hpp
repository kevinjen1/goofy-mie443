#pragma once

#include<ros/ros.h>
#include<geometry_msgs/PoseStamped.h>
#include<sensor_msgs/LaserScan.h>
#include<geometry_msgs/Point.h>

namespace goofy{
namespace mapper{

class LaserMap{
public:
	LaserMap(){}

	bool checkPosition(geometry_msgs::PoseStamped pose);
private:
	void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

	std::vector<geometry_msgs::Point> obstacle_positions;

	ros::NodeHandle _nh;
	ros::Subscriber _laser_sub;


};

}
}

