#include <ros/console.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/LaserScan.h>
#include <eStop.h>

#include <stdio.h>
#include <cmath>
#include"mapper/mapper.hpp"
#include"planner/primitive_planner.hpp"
#include"common/common.hpp"

using namespace std;
using namespace goofy;

void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg){
	//fill with your code
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
	//fill with your code
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "image_listener");
	ros::NodeHandle nh;
	teleController eStop;

	ros::Subscriber bumper_sub = nh.subscribe("mobile_base/events/bumper", 10, &bumperCallback);
	ros::Subscriber laser_sub = nh.subscribe("scan", 10, &laserCallback);

	ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);

	//Setup Robot
	common::RobotModel robot(0.5,0.5,0.5);

	//Setup Primitives
	common::BasicMotion straight{1, 0, 6000};
	common::BasicMotion turn_left{1,0.3, 6000};
	common::BasicMotion turn_right{1.,-0.3, 6000};

	planner::MotionList motions;
	motions.push_back(straight);
	motions.push_back(turn_left);
	motions.push_back(turn_right);

	//Setup Planner
	planner::PrimitiveRepresentation primitives(robot, motions);
	planner::RandomPlanner random_planner(primitives);
	common::Visualizer vis;

	double angular = 0.0;
	double linear = 0.0;
	geometry_msgs::Twist vel;

	random_planner.runIteration();

	while(ros::ok()){
		ros::spinOnce();
		//.....**E-STOP DO NOT TOUCH**.......
		eStop.block();
		//...................................

		//fill with your code
		nav_msgs::Path path = random_planner.getPath();
		vis.publishPath(path, std::chrono::milliseconds(10));
		if (!random_planner.getVelocity(vel)){
			random_planner.runIteration();
			std::cout << "Getting new plan!" << std::endl;
		}

 		vel_pub.publish(vel);
	}

	return 0;
}
