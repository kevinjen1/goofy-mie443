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
		sub = n.subscribe("goofMap", 1, &FindCoord::callback, this);
	}
	void callback(const nav_msgs::OccupancyGrid grid) {
		// need a position
		// have an A* search, without going the way we came
		// need to have a list of past places

		// get current position from odometry
		double currentX, currentY;

		// need to get odom. probably from a service
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
private:
	ros::NodeHandle n;
	ros::Publisher pub;
	ros::Subscriber sub;
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
