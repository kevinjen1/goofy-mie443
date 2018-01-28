#pragma once

#include<ros/ros.h>
#include<geometry_msgs/PoseStamped.h>
#include<sensor_msgs/LaserScan.h>
#include<geometry_msgs/Point.h>
#include<nav_msgs/OccupancyGrid.h>
#include<vector>

using namespace std;

namespace goofy{
namespace mapper{

class LaserMap{
public:
	LaserMap(){}

	bool checkPosition(geometry_msgs::PoseStamped pose);
private:
	void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

	vector<geometry_msgs::Point> obstacle_positions;

	ros::NodeHandle _nh;
	ros::Subscriber _laser_sub;
};

struct LocalMap {
	// map
	vector<vector<int> > matrix;
	double resolution;
	int boundingHeight;
	int boundingWidth;
	geometry_msgs::Pose origin;
	ros::Time time;
};

class GMap{
public:
	GMap(){}
	LocalMap processRawMap(const nav_msgs::OccupancyGrid grid);
private:
	int getMinHeight(int width, int height, std::vector<int8_t> data);
	int getMaxHeight(int width, int height, std::vector<int8_t> data);
	int getMinWidth(int width, int height, std::vector<int8_t> data);
	int getMaxWidth(int width, int height, std::vector<int8_t> data);
};

}
}

