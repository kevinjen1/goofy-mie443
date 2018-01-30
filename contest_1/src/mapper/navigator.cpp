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

//void lookForCoordinates(const nav_msgs::OccupancyGrid grid) {
void lookForCoordinates(const LocalMap grid) {
	// need a position
	// have an A* search, without going the way we came
	// need to have a list of past places

	// get current position from odometry
	double currentX, currentY;

	nav_msgs::Odometry odom;
	geometry_msgs::Pose2D origin;
	geometry_msgs::Pose2D robotPos;

	robotPos.x = odom.pose.pose.position.x;
	robotPos.y = odom.pose.pose.position.y;
	tf::Quaternion q (odom.pose.pose.orientation.x,
			odom.pose.pose.orientation.y,
			odom.pose.pose.orientation.z,
			odom.pose.pose.orientation.w
			);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	robotPos.theta = goofy::common::quat2yaw(odom.pose.pose.orientation);

	origin.x = grid.origin.position.x;
	origin.y = grid.origin.position.y;
	origin.theta = goofy::common::quat2yaw(grid.origin.orientation);

	// THE ANGLES FOR THESE TWO LINES ARE PROBABLY NOT
	int diffX = (robotPos.x - origin.x) * cos(origin.theta);
	int diffY = (robotPos.y - origin.y) * sin(origin.theta);

	int row = diffY/grid.resolution;
	int col = diffX/grid.resolution;
//
	geometry_msgs::Pose2D coord = getCoordinate(grid);

	//publish coordinate
	coord_pub.publish(coord);

}

geometry_msgs::Pose2D getCoordinate(goofy::mapper::LocalMap grid) {

	// use A* to find the nearest cell with -1 in the grid

	geometry_msgs::Pose2D coord;// = new geometry_msgs::Pose2D;
	return coord;
}
}}


int main(int argc, char **argv) {
	ros::init(argc, argv, "goofNavigator");
	ros::NodeHandle n;

	ROS_INFO("Here is goof navigator");
	goofy::mapper::LocalMap hello;

	ros::Subscriber mapper_sub =  n.subscribe("goofMap", 1000, &goofy::mapper::lookForCoordinates);
	goofy::mapper::coord_pub = n.advertise<geometry_msgs::Pose2D>("goofCoord", 1000);
//
////	ros::Rate loop_rate(10);
//	ros::spin();
}
