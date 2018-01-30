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
		//nav_msgs::Odometry odom;
		geometry_msgs::Pose2D origin;

		origin.x = grid.info.origin.position.x;
		origin.y = grid.info.origin.position.y;
		origin.theta = goofy::common::quat2yaw(grid.info.origin.orientation);

		// THE ANGLES FOR THESE TWO LINES ARE PROBABLY NOT
		int diffX = (robotPos.x - origin.x) * cos(origin.theta);
		int diffY = (robotPos.y - origin.y) * sin(origin.theta);

		int row = diffY/grid.info.resolution;
		int col = diffX/grid.info.resolution;

		geometry_msgs::Pose2D coord = getCoordinate(grid);

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


geometry_msgs::Pose2D getCoordinate(nav_msgs::OccupancyGrid grid) {

	// use A* to find the nearest cell with -1 in the grid

	geometry_msgs::Pose2D coord;// = new geometry_msgs::Pose2D;
	return coord;
}
}}


int main(int argc, char **argv) {
	ros::init(argc, argv, "goofNavigator");
	goofy::mapper::FindCoord subAndPub;

	ROS_INFO("goofNavigator is running");
	ros::spin();
	return 0;
}
