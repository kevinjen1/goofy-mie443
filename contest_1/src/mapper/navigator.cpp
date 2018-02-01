#include "ros/ros.h"
//#include "mapper/mapper.hpp"
#include "common/common.hpp"
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include "mapper/navigator.hpp"

/*
 * This should be a subscriber to mapper,
 * and should publish to a topic
 */


namespace goofy{
namespace mapper{

class FindCoord {
public:
	FindCoord() {
		pub = n.advertise<geometry_msgs::Pose2D>("goofCoord",1);
		subMap = n.subscribe("goofMap", 1, &FindCoord::callbackMap, this);
		subOdom = n.subscribe("odom", 1, &FindCoord::callbackOdom, this);

		// Set robot's initial pose
		robotPos.x = 0.0f;
		robotPos.y = 0.0f;
		robotPos.theta = 0.0f;
	}
	void callbackMap(const nav_msgs::OccupancyGrid grid) {
		// need a position
		// have an A* search, without going the way we came
		// need to have a list of past places

		// get current position from odometry
		double currentX, currentY;

		// need to get odom. probably from a service
		geometry_msgs::Pose2D origin;

		origin.x = grid.info.origin.position.x;
		origin.y = grid.info.origin.position.y;
		origin.theta = goofy::common::quat2yaw(grid.info.origin.orientation);

		// ASSUMING THAT MAP DOES NOT ROTATE
		int diffX = (robotPos.x - origin.x);
		int diffY = (robotPos.y - origin.y);

		Slope slope = getClosestAxisToHeading(robotPos.theta);

		int row = diffY/grid.info.resolution;
		int col = diffX/grid.info.resolution;

		geometry_msgs::Pose2D coord = getCoordinateRayCasting(grid, slope, row, col, robotPos);

		// couldn't retrieve an unknown location
		if (coord.x == 0 && coord.y == 0) {
			return;
		}

		//publish coordinate
		pub.publish(coord);
	}
	void callbackOdom(const nav_msgs::Odometry odom)
	{
		// Find robot 2D pose from odometry
		// Position (x,y)
		robotPos.x = odom.pose.pose.position.x;
		robotPos.y = odom.pose.pose.position.y;

		robotPos.theta = goofy::common::quat2yaw(odom.pose.pose.orientation);
	}

private:
	ros::NodeHandle n;
	ros::Publisher pub;
	ros::Subscriber subMap;
	ros::Subscriber subOdom;

	geometry_msgs::Pose2D robotPos;
};

vector<vector<int>> getMatrixFromGrid(nav_msgs::OccupancyGrid grid) {
	int width = grid.info.width;
	int height = grid.info.height;
	vector<vector<int>> matrix;
	matrix.resize(height, vector<int>(width));
	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			int index = i * width + j;
			matrix[i][j] = grid.data[index];
		}
	}
	return matrix;
}


//// doesn't work
//int** get2dArrayFromGrid(nav_msgs::OccupancyGrid grid) {
//	int width = grid.info.width;
//	int height = grid.info.height;
//	int matrix[height][width];
//	for (int i = 0; i < height; i++) {
//		for (int j = 0; j < width; j++) {
//			int index = i * width + j;
//			matrix[i][j] = grid.data[index];
//		}
//	}
//	return matrix;
//}

Slope getClosestAxisToHeading(double theta)
{
	// Find closest coordinate axis to heading: theta
	// Run index 0, Rise index 1
	double x = cos(theta);
	double y = sin(theta);

	double xAbs = abs(cos(theta));
	double yAbs = abs(sin(theta));

	Slope slope;

	if (xAbs >= yAbs)
	{
		x > 0 ? slope.run = 1: slope.run = -1;
	}
	else
	{
		y > 0 ? slope.rise = 1: slope.rise = -1;
	}

	return slope;
}

/**
 * Get an unknown coordinate of the map to explore
 */
geometry_msgs::Pose2D getCoordinateRayCasting(nav_msgs::OccupancyGrid grid, Slope slope, int robotRow, int robotCol, geometry_msgs::Pose2D robotPos) {

	vector<vector<int>> matrix = getMatrixFromGrid(grid);

	// use depth search to find the nearest cell with -1 in the grid
	double run = slope.run;
	double rise = slope.rise;

	int height = matrix.size();
	int width = matrix[0].size();

	int row = robotRow;
	int col = robotCol;

	double tenDegrees = 0.1745;
	int angleChange = 0;
	int cellState = 0;
	int angle = 0;

	while (angle < PI) {
		int step = 1;
		while (true) {
			row = robotRow + (rise * step);
			col = robotCol + (run * step);
			// out of bounds
			if (row >= height || col >= width || row < 0 || col < 0) {
				break;
			}
			cellState = matrix[row][col];
			// either it's occupied or unknown. Either way, ray casting stops
			if (cellState > 50 || cellState < 0) {
				break;
			}
			step++;
		}
		// found an unknown location
		if (cellState == -1) {
			break;
		}

		angle = getAngle(&angleChange);

		// rotate the heading by 10 degrees
		if (slope.run == 0) {
			rise = cos(angle) * slope.rise;
			run = sin(angle);
		} else if (slope.rise == 0) {
			run = cos(angle) * slope.run;
			rise = sin(angle);
		}
	}

	geometry_msgs::Pose2D coord;

	int degreeAngle = angle * 180/PI;
	ROS_INFO_STREAM("angle: " << degreeAngle);
	ROS_INFO_STREAM("originalRunRise: " << slope.run << "/" << slope.rise);
	ROS_INFO_STREAM("finalRunRise: " << run << "/" << rise);
	ROS_INFO_STREAM("finalRunRise: " << run << "/" << rise);
	ROS_INFO_STREAM("RobotPos: " << robotPos.x << "," << robotPos.y);


	// didn't find a cell to discover
	if (cellState >= 0) {
		ROS_INFO_STREAM("COULD NOT FIND COORD");
		return coord;
	}

	coord.x = grid.info.origin.position.x + (col * grid.info.resolution);
	coord.y = grid.info.origin.position.y + (row * grid.info.resolution);

	ROS_INFO_STREAM("PickedPos: " << coord.x << "," << coord.y);

	return coord;
}

/**
 * Get the new angle given the stage of incremental change
 */
int getAngle(int* angleChange) {
	double tenDegrees = 0.1745;

	if (*angleChange == 0) {
		*angleChange = 1;
	} else if (*angleChange > 0) {
		*angleChange = -*angleChange;
	} else {
		*angleChange = -*angleChange + 1;
	}
	return tenDegrees*(*angleChange);
}

}}


int main(int argc, char **argv) {
	ros::init(argc, argv, "goofNavigator");
	goofy::mapper::FindCoord subAndPub;

	ROS_INFO("goofNavigator is running");
	ros::spin();
	return 0;
}
