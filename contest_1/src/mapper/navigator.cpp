#include "ros/ros.h"
//#include "mapper/mapper.hpp"
#include "common/common.hpp"
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include "mapper/navigator.hpp"
#include <stack>
#include <list>

/*
 * This should be a subscriber to mapper,
 * and should publish to a topic
 */


namespace goofy{
namespace mapper{

ros::Time lastChecked;
nav_msgs::OccupancyGrid lastGrid;
bool isGridInitialized = false;
int refreshInterval = 40;
const double MIN_DISTANCE_PUBLISH = 0.5;


class FindCoord {
public:
	FindCoord() {
		pub = n.advertise<geometry_msgs::Pose2D>("goofCoord",1);
		subMap = n.subscribe("goofMap", 1, &FindCoord::callbackMap, this);
//		subOdom = n.subscribe("odom", 1, &FindCoord::callbackOdom, this);
		marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
		timer = n.createTimer(ros::Duration(5), &FindCoord::timerCallback, this);

		// Set robot's initial pose
		robotPos.x = 0.0f;
		robotPos.y = 0.0f;
		robotPos.theta = 0.0f;
	}
	void callbackMap(const nav_msgs::OccupancyGrid grid) {
		tf::StampedTransform transform;
		try{
		  listener.lookupTransform("/map", "/base_link",
								   ros::Time(0), transform);
		  ROS_INFO_STREAM("Got the transform");
		}
		catch (tf::TransformException ex){
		  ROS_ERROR("%s",ex.what());
		  ros::Duration(1.0).sleep();
		}
		ROS_INFO_STREAM("transform pos: "<< transform.getOrigin().x() << ", " << transform.getOrigin().y());

		// need a position
		// have an A* search, without going the way we came
		// need to have a list of past places

		lastChecked = ros::Time::now();
		lastGrid = grid;
		isGridInitialized = true;

		// get current position from odometry
		double currentX, currentY;

		// need to get odom. probably from a service
		geometry_msgs::Pose2D origin;
		geometry_msgs::Pose2D robPos;
//		robPos = robotPos;
		robPos.x = transform.getOrigin().x();
		robPos.y = transform.getOrigin().y();

		geometry_msgs::Quaternion quat;
		quat.x = transform.getRotation().x();
		quat.y = transform.getRotation().y();
		quat.z = transform.getRotation().z();
		quat.w = transform.getRotation().w();

		robPos.theta = common::quat2yaw(quat);

		ROS_INFO_STREAM("robot pos: "<< robotPos.x << ", "<< robotPos.y << ", " << robotPos.theta);

		origin.x = grid.info.origin.position.x;
		origin.y = grid.info.origin.position.y;
		origin.theta = goofy::common::quat2yaw(grid.info.origin.orientation);

		// ASSUMING THAT MAP DOES NOT ROTATE
		int diffX = (robPos.x - origin.x);
		int diffY = (robPos.y - origin.y);

		Slope slope = getClosestAxisToHeading(robPos.theta);

		int row = diffY/grid.info.resolution;
		int col = diffX/grid.info.resolution;

		geometry_msgs::Pose2D coord = getCoordinateRayCasting(grid, slope, row, col, robPos, &publishedPts);
//		geometry_msgs::Pose2D coord = getCoordinateBFS(grid, slope, row, col, robPos);

		// couldn't retrieve an unknown location
		if (isZero(coord.x) && isZero(coord.y)) {
			return;
		}

		geometry_msgs::Pose2D localCoord = convertToLocal(coord, robPos);

		//publish coordinate
		//pub.publish(localCoord);
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

	geometry_msgs::Pose2D convertToLocal(geometry_msgs::Pose2D coord, geometry_msgs::Pose2D robPos) {
		geometry_msgs::Pose2D localCoord;
		double deltaX = coord.x - robPos.x;
		double deltaY = coord.y - robPos.y;

		localCoord.x = deltaX*cos(robPos.theta) + deltaY*sin(robPos.theta);
		localCoord.y = -deltaX*sin(robPos.theta) + deltaY*cos(robPos.theta);
		return localCoord;
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

//		ROS_INFO_STREAM("Num: "<<checkedPts.size());

//		for (int i=0; i<checkedPts.size(); i++) {
//			geometry_msgs::Point p;
//			p.x = checkedPts[i].x;
//			p.y = checkedPts[i].y;
//			p.z = 0;
//
//			points.points.push_back(p);
//		}

		geometry_msgs::Point p;
		p.x = coord.x;
		p.y = coord.y;
		p.z = 0;

		points.points.push_back(p);

		marker_pub.publish(points);
	}

	void timerCallback(const ros::TimerEvent& event) {
//		ROS_INFO_STREAM("in timer");

		ros::Duration duration = ros::Time::now() - lastChecked;
		if (!isGridInitialized) {
//			ROS_INFO_STREAM("exiting timer");
			return;
		}

		if (duration.toSec() > refreshInterval) {
			ROS_INFO_STREAM("HIIIIIIIIII");
			callbackMap(lastGrid);
//			ROS_INFO_STREAM("I don't know why i'm getting seg fault");
		}
//		ROS_INFO_STREAM("I am here for timer");
	}

private:
	ros::NodeHandle n;
	ros::Publisher pub;
	ros::Subscriber subMap;
//	ros::Subscriber subOdom;
	ros::Publisher marker_pub;

	tf::TransformListener listener;

	ros::Timer timer;

	geometry_msgs::Pose2D robotPos;
	vector<geometry_msgs::Pose2D> publishedPts;
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
//	double x = cos(theta);
//	double y = sin(theta);
//
//	double xAbs = abs(cos(theta));
//	double yAbs = abs(sin(theta));
//
//	Slope slope;
//
//	if (xAbs >= yAbs)
//	{
//		x > 0 ? slope.run = 1: slope.run = -1;
//	}
//	else
//	{
//		y > 0 ? slope.rise = 1: slope.rise = -1;
//	}
//
//	return slope;

	Slope slope;
	slope.run = cos(theta);
	slope.rise = sin(theta);

	return slope;
}

/**
 * Get an unknown coordinate of the map to explore
 */
geometry_msgs::Pose2D getCoordinateRayCasting(nav_msgs::OccupancyGrid grid, Slope slope, int robotRow, int robotCol, geometry_msgs::Pose2D robotPos, vector<geometry_msgs::Pose2D> *publishedPts) {

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
	double deltaAngle = 0;

//	ROS_INFO_STREAM("starting");
	geometry_msgs::Pose2D coord;

	while (deltaAngle < PI) {
		double step = 0.5;
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
			step += 0.5;
		}
		// found an unknown location
		if (cellState == -1) {
//			ROS_INFO_STREAM("FOUND AN UNKNOWN LOCATION");

			// get the coordinate
			coord = getCoordinateFromMap(row, col, grid);
			int publishedSize = (*publishedPts).size();
//			ROS_INFO_STREAM("Published size: " << publishedSize);
			bool isClosePoint = false;

			// make sure it's not close to the last two picked points
			if (publishedSize > 0) {
				geometry_msgs::Pose2D lastPt = (*publishedPts)[publishedSize - 1];
				float dist = getDistance(coord, lastPt);
//				ROS_INFO_STREAM("dist1:" << dist);
				if (dist < MIN_DISTANCE_PUBLISH) {
					ROS_INFO_STREAM("POINT CLOSE to last point: " << dist);
					coord.x = 0;
					coord.y = 0;
					isClosePoint = true;
				}
			}
			if (publishedSize > 1) {
				geometry_msgs::Pose2D lastPt = (*publishedPts)[publishedSize - 2];
				float dist = getDistance(coord, lastPt);
//				ROS_INFO_STREAM("dist2:" << dist);
				if (dist < MIN_DISTANCE_PUBLISH) {
					ROS_INFO_STREAM("POINT CLOSE to second last point: " << dist);
					coord.x = 0;
					coord.y = 0;
					isClosePoint = true;
				}
			}
			if (!isClosePoint) {
				// found a point
				break;
			}
		}
//		ROS_INFO_STREAM("MOVING ON");

		deltaAngle = getAngle(&angleChange);
		ROS_INFO_STREAM("trying angle: " << deltaAngle << " - " << convertToDegree(deltaAngle));

//		ROS_INFO_STREAM("pushing a point: " << checkedPts.size());
//		checkedPts.push_back(getCoordinateFromMap(row, col, grid));
//		ROS_INFO_STREAM("done: " << checkedPts.size());

		run = cos(deltaAngle + slope.angle);
		rise = sin(deltaAngle + slope.angle);


//		// rotate the heading by 10 degrees
//		if (isZero(slope.run)) {
//			ROS_INFO_STREAM("run is zero");
//			rise = cos(deltaAngle) * slope.rise;
//			run = sin(deltaAngle);
//		} else if (isZero(slope.rise)) {
//			ROS_INFO_STREAM("rise is zero");
//			run = cos(deltaAngle) * slope.run;
//			rise = sin(deltaAngle);
//		}
	}

	ROS_INFO_STREAM("angle: " << convertToDegree(deltaAngle));
//	ROS_INFO_STREAM("originalRunRise: " << slope.run << "/" << slope.rise);
//	ROS_INFO_STREAM("finalRunRise: " << run << "/" << rise);
	ROS_INFO_STREAM("RobotPos: " << robotPos.x << "," << robotPos.y);


	// didn't find a cell to discover
	if (cellState >= 0 || (isZero(coord.x) && isZero(coord.y))) {
		ROS_INFO_STREAM("COULD NOT FIND COORD");
		return coord;
	}

	ROS_INFO_STREAM("PickedPos: " << coord.x << "," << coord.y);
//	ROS_INFO_STREAM("sizeBefore:" << (*publishedPts).size());
	(*publishedPts).push_back(coord);
//	ROS_INFO_STREAM("sizeAfter:" << (*publishedPts).size());

	return coord;
}

float getDistance(geometry_msgs::Pose2D pt1, geometry_msgs::Pose2D pt2) {
	return sqrt(pow((pt1.x - pt2.x), 2) + pow((pt1.y - pt2.y),2) );
}

geometry_msgs::Pose2D getCoordinateFromMap(int row, int col, nav_msgs::OccupancyGrid grid) {
	geometry_msgs::Pose2D coord;
	coord.x = grid.info.origin.position.x + (col * grid.info.resolution);
	coord.y = grid.info.origin.position.y + (row * grid.info.resolution);

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

// DFS Functions
/*
	Make grid for DFS using OccupancyGrid
 */
vector<vector<gridDFSElement>> getMatrixFromGridDFS(nav_msgs::OccupancyGrid grid, Slope slope)
{
	int width = grid.info.width;
	int height = grid.info.height;

	vector<vector<gridDFSElement>> matrix;
	matrix.resize(height, vector<gridDFSElement>(width));
	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			int index = i * width + j;
			matrix[i][j].data = grid.data[index];
			matrix[i][j].index.row = i;
			matrix[i][j].index.col = j;
			matrix[i][j].isDiscovered = false;

			// Dont care about neighbors for obstacles
			if (matrix[i][j].data < 50)
			{
				matrix[i][j].adjList = fillAdjList(matrix, i, j, slope, height, width);
			}
		}
	}

	return matrix;
}

/*
	Create an array of neighbors for each element in the grid
 */
vector<gridIndex> fillAdjList(vector<vector<gridDFSElement>> matrix, int i, int j, Slope slope, int height, int width)
{
	vector<gridIndex> v;
	// Fill Adj List in CCW Direction
	// Prioritize rise
	if (slope.run == 0)
	{
		// Pos Y Axis
		if (slope.rise == 1)
		{
			// Pos Y Direction First
			pushBackPosY(v, i, j, height);

			pushBackPosX(v, i, j, width);
			pushBackNegY(v, i, j);
			pushBackNegX(v, i, j);
		}
		// Neg Y Axis
		else
		{
			// Neg Y Direction First
			pushBackNegY(v, i, j);

			pushBackNegX(v, i, j);
			pushBackPosY(v, i, j, height);
			pushBackPosX(v, i, j, width);
		}
	}
	// Prioritize run
	else
	{
		// Pos X Axis
		if (slope.run == 1)
		{
			// Pos X Direction First
			pushBackPosX(v, i, j, width);

			pushBackNegY(v, i, j);
			pushBackNegX(v, i, j);
			pushBackPosY(v, i, j, height);
		}
		// Neg X axis
		else
		{
			// Neg X Direction First
			pushBackNegX(v, i, j);

			pushBackPosY(v, i, j, height);
			pushBackPosX(v, i, j, width);
			pushBackNegY(v, i, j);
		}
	}

	return v;
}

/*
	Helper functions for filling adjaceny matrix
 */
void pushBackNegX(vector<gridIndex> &v, int i, int j)
{
	if (j - 1 >= 0)
		v.push_back(gridIndex(i, j - 1));
}

void pushBackPosX(vector<gridIndex> &v, int i, int j, int width)
{
	if (j + 1 < width)
		v.push_back(gridIndex(i, j + 1));
}

void pushBackNegY(vector<gridIndex> &v, int i, int j)
{
	if (i - 1 >= 0)
		v.push_back(gridIndex(i - 1, j));
}

void pushBackPosY(vector<gridIndex> &v, int i, int j, int height)
{
	if (i + 1 < height)
		v.push_back(gridIndex(i + 1, j));
}

/*
* Get an unknown coordinate of the map to explore
*/
geometry_msgs::Pose2D getCoordinateDFS(nav_msgs::OccupancyGrid grid, Slope slope, int robotRow, int robotCol, geometry_msgs::Pose2D t)
{
	stack<gridDFSElement> stack;

	vector<vector<gridDFSElement>> matrix = getMatrixFromGridDFS(grid, slope);

	gridDFSElement v = matrix[robotRow][robotCol];

	stack.push(v);

	geometry_msgs::Pose2D coord;
	// Set coord to 0
	coord.x = 0.0f;
	coord.y = 0.0f;

	while (!stack.empty())
	{
		v = stack.top();

		// Check if found an unknown
		if (v.data == -1)
		{
			ROS_INFO_STREAM("found uknown");
			coord.x = grid.info.origin.position.x + (v.index.col * grid.info.resolution);
			coord.y = grid.info.origin.position.y + (v.index.row * grid.info.resolution);
			break;
		}

		stack.pop();

		if (matrix[v.index.row][v.index.col].isDiscovered)
			continue;

		matrix[v.index.row][v.index.col].isDiscovered = true;

		gridIndex neighborIndex;
		for (auto it = v.adjList.begin(); it != v.adjList.end(); ++it)
		{
			neighborIndex = *it;
			if (!matrix[neighborIndex.row][neighborIndex.col].isDiscovered && matrix[neighborIndex.row][neighborIndex.col].data < 50)
				stack.push(matrix[neighborIndex.row][neighborIndex.col]);
		}
	}
	ROS_INFO_STREAM("point picked: " << coord.x << ", " << coord.y);

	return coord;
}

geometry_msgs::Pose2D getCoordinateBFS(nav_msgs::OccupancyGrid grid, Slope slope, int robotRow, int robotCol, geometry_msgs::Pose2D t)
{
	list<gridDFSElement> queue;

	vector<vector<gridDFSElement>> matrix = getMatrixFromGridDFS(grid, slope);

	gridDFSElement v = matrix[robotRow][robotCol];

	queue.push_back(v);

	geometry_msgs::Pose2D coord;
	// Set coord to 0
	coord.x = 0.0f;
	coord.y = 0.0f;

	while (!queue.empty())
	{
		v = queue.front();

		// Check if found an unknown
		if (v.data == -1)
		{
			ROS_INFO_STREAM("found uknown");
			coord.x = grid.info.origin.position.x + (v.index.col * grid.info.resolution);
			coord.y = grid.info.origin.position.y + (v.index.row * grid.info.resolution);
			break;
		}

		queue.pop_front();

		if (matrix[v.index.row][v.index.col].isDiscovered)
			continue;

		matrix[v.index.row][v.index.col].isDiscovered = true;

		gridIndex neighborIndex;
		for (auto it = v.adjList.begin(); it != v.adjList.end(); ++it)
		{
			neighborIndex = *it;
			if (!matrix[neighborIndex.row][neighborIndex.col].isDiscovered && matrix[neighborIndex.row][neighborIndex.col].data < 50)
				queue.push_back(matrix[neighborIndex.row][neighborIndex.col]);
		}
	}
	ROS_INFO_STREAM("point picked: " << coord.x << ", " << coord.y);

	return coord;
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

}}


int main(int argc, char **argv) {
	ros::init(argc, argv, "goofNavigator");
	goofy::mapper::FindCoord subAndPub;

	ROS_INFO("goofNavigator is running");
	ros::spin();
	return 0;
}
