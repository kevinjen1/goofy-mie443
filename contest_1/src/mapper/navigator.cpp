#include "ros/ros.h"
//#include "mapper/mapper.hpp"
#include "common/common.hpp"
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <visualization_msgs/Marker.h>
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
		marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);

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
		if (isZero(coord.x) && isZero(coord.y)) {
			return;
		}

		//publish coordinate
		pub.publish(coord);
		publishToRviz(coord);
	}
	void callbackOdom(const nav_msgs::Odometry odom)
	{
		// Find robot 2D pose from odometry
		// Position (x,y)
		robotPos.x = odom.pose.pose.position.x;
		robotPos.y = odom.pose.pose.position.y;

		robotPos.theta = goofy::common::quat2yaw(odom.pose.pose.orientation);
	}

	void publishToRviz(geometry_msgs::Pose2D coord) {
		ROS_INFO_STREAM("publishing to RVIZ");
		visualization_msgs::Marker points;
		points.header.frame_id = "map";
		points.header.stamp = ros::Time::now();
		points.ns = "points_and_lines";
		points.action = visualization_msgs::Marker::ADD;
		points.pose.orientation.w = 1.0;

		points.id = 0;

		points.type = visualization_msgs::Marker::POINTS;

		// POINTS markers use x and y scale for width/height respectively
		points.scale.x = 0.2;
		points.scale.y = 0.2;

		// Points are green
		points.color.g = 1.0f;
		points.color.a = 1.0;

		geometry_msgs::Point p;
		p.x = coord.x;
		p.y = coord.y;
		p.z = 0;

		points.points.push_back(p);

		marker_pub.publish(points);
	}

private:
	ros::NodeHandle n;
	ros::Publisher pub;
	ros::Subscriber subMap;
	ros::Subscriber subOdom;
	ros::Publisher marker_pub;

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
	double angle = 0;

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
		ROS_INFO_STREAM("trying angle: " << angle << " - " << convertToDegree(angle));

		// rotate the heading by 10 degrees
		if (isZero(slope.run)) {
			ROS_INFO_STREAM("run is zero");
			rise = cos(angle) * slope.rise;
			run = sin(angle);
		} else if (isZero(slope.rise)) {
			ROS_INFO_STREAM("rise is zero");
			run = cos(angle) * slope.run;
			rise = sin(angle);
		}
	}

	geometry_msgs::Pose2D coord;


	ROS_INFO_STREAM("angle: " << convertToDegree(angle));
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
double getAngle(int* angleChange) {
	double tenDegrees = 0.1745;
	ROS_INFO_STREAM("Angle change is " << (*angleChange));

	if (*angleChange == 0) {
		*angleChange = 1;
	} else if (*angleChange > 0) {
		*angleChange = -*angleChange;
	} else {
		*angleChange = -*angleChange + 1;
	}
	ROS_INFO_STREAM("changed to: " << (*angleChange));
	return tenDegrees*(*angleChange);
}

bool isZero(double value) {
	double sigma = 1e-3;
	return abs(value) - 0 < sigma;
}

int convertToDegree(double rad) {
	return rad * 180/PI;
}

//vector<vector<gridDFSElement>> getMatrixFromGridDFS(
//		nav_msgs::OccupancyGrid grid, Slope slope) {
//
//	int width = grid.info.width;
//	int height = grid.info.height;
//	vector<vector<gridDFSElement> > matrix;
//	matrix.resize(height, vector<gridDFSElement>(width));
//	for (int i = 0; i < height; i++) {
//		for (int j = 0; j < width; j++) {
//			int index = i * width + j;
//			matrix[i][j].data = grid.data[index];
//			matrix[i][j].index.row = i;
//			matrix[i][j].index.col = j;
//			matrix[i][j].isDiscovered = false;
//
//			// Dont care about neighbors for obstacles
//			if (matrix[i][j].data < 50) {
//				matrix[i][j].adjList = fillAdjList(matrix, i, j, slope, height,
//						width);
//			}
//		}
//	}
//
//	return matrix;
//}
//
//void fillAdjList(vector<vector<gridDFSElement>> matrix, int i, int j,
//		Slope slope, int height, int width) {
//	// Fill Adj List in CCW Direction
//	// Prioritize rise
//	if (slope.run == 0) {
//		// Pos Y Axis
//		if (slope.rise == 1) {
//			// Pos Y Direction First
//			pushBackPosY(i, j, height);
//
//			pushBackPosX(i, j, width);
//			pushBackNegY(i, j);
//			pushBackNegX(i, j);
//		}
//		// Neg Y Axis
//		else {
//			// Neg Y Direction First
//			pushBackNegY(i, j);
//
//			pushBackNegX(i, j);
//			pushBackPosY(i, j, height);
//			pushBackPosX(i, j, width);
//		}
//	}
//	// Prioritize run
//	else {
//		// Pos X Axis
//		if (slope.run == 1) {
//			// Pos X Direction First
//			pushBackPosX(i, j, width);
//
//			pushBackNegY(i, j);
//			pushBackNegX(i, j);
//			pushBackPosY(i, j, height);
//		}
//		// Neg X axis
//		else {
//			// Neg X Direction First
//			pushBackNegX(i, j);
//
//			pushBackPosY(i, j, height);
//			pushBackPosX(i, j, width);
//			pushBackNegY(i, j);
//		}
//	}
//}
//
//void pushBackNegX(vector<gridIndex> &v, int i, int j) {
//	if (j - 1 >= 0)
//		v.push_back(gridIndex(i, j - 1));
//}
//
//void pushBackPosX(vector<gridIndex> &v, int i, int j, int width) {
//	if (j + 1 < width)
//		v.push_back(gridIndex(i, j + 1));
//}
//
//void pushBackNegY(vector<gridIndex> &v, int i, int j) {
//	if (i - 1 >= 0)
//		v.push_back(gridIndex(i - 1, j));
//}
//
//void pushBackPosY(vector<gridIndex> &v, int i, int j, int height) {
//	if (i + 1 < height)
//		v.push_back(gridIndex(i + 1, j));
//}
//
///*
// * Get an unknown coordinate of the map to explore
// */
//geometry_msgs::Pose2D getCoordinateDFS(nav_msgs::OccupancyGrid grid,
//		Slope slope, int robotRow, int robotCol) {
//	stack<gridDFSElement> stack;
//
//	vector<vector<gridDFSElement>> matrix = getMatrixFromGrid(grid, slope);
//
//	gridDFSElement v = matrix[robotRow][robotCol];
//
//	stack.push(v);
//
//	geometry_msgs::Pose2D coord;
//
//	while (!stack.empty()) {
//		v = stack.top();
//
//		// Check if found an unknown
//		if (v.data == -1) {
//			coord.x = grid.info.origin.position.x
//					+ (v.col * grid.info.resolution);
//			coord.y = grid.info.origin.position.y
//					+ (v.row * grid.info.resolution);
//			break;
//		}
//
//		stack.pop();
//
//		if (v.isDiscovered)
//			continue;
//
//		v.isDiscovered = true;
//
//		gridIndex neighborIndex;
//
//		for (auto it = v.adjList.begin(); it != v.adjList.end(); ++it) {
//			int u = *it;
//			neighborIndex = v.adjList[u];
//
//			if (!matrix[neighborIndex.row][neighborIndex.col].isDiscovered
//					&& matrix[neighborIndex.row][neighborIndex.col].data < 50)
//				stack.push(matrix[neighborIndex.row][neighborIndex.col]);
//		}
//	}
//
//	return coord;
//}


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

}}


int main(int argc, char **argv) {
	ros::init(argc, argv, "goofNavigator");
	goofy::mapper::FindCoord subAndPub;

	ROS_INFO("goofNavigator is running");
	ros::spin();
	return 0;
}
