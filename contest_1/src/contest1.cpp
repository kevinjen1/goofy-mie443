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

bool bumperL = 0, bumperC = 0, bumperR = 0;
double lRange = 10;
int lSize = 0, lOffset = 0, dAngle = 5;
sensor_msgs::LaserScan::ConstPtr curr_scan;

void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg){
	if(msg->bumper == 0)
		bumperL = !bumperL;
	else if(msg->bumper == 1)
		bumperC = !bumperC;
	else if(msg->bumper == 2)
		bumperR = !bumperR;
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){

	// Print statement to test checkObstacle() - remove later
	// std::cout << "Check Obstacle:" << goofy::planner::PrimitivePlanner::checkObstacle(msg, 1, 0.25) << endl;
	
	lSize = (msg->angle_max -msg->angle_min)/msg->angle_increment;
	lOffset = dAngle*Pi/(180*msg->angle_increment);
	lRange = 11;
	
	if (dAngle*Pi/180 < msg->angle_max && -dAngle*Pi/180 > msg->angle_min){
		for (int i = lSize/2 - lOffset; i < lSize/2 + lOffset; i++){
			if (lRange > msg->ranges[i])
				lRange = msg->ranges[i];
		}
	}
	else{
		for (int i = 0; i < lSize; i++){
			if (lRange > msg->ranges[i])
				lRange = msg->ranges[i];
		}
	}

	if (lRange == 11){
		lRange = 0;
	}

	curr_scan = msg;
	return;
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
	common::BasicMotion on_spot{0, 0.3, 6000};

	planner::MotionList motions;
	motions.push_back(straight);
	motions.push_back(turn_left);
	motions.push_back(turn_right);
	motions.push_back(on_spot);

	//Setup Planner
	planner::PrimitiveRepresentation primitives(robot, motions);
	planner::RandomPlanner random_planner(primitives);
	common::Visualizer vis;

	double angular = 0.0;
	double linear = 0.0;
	geometry_msgs::Twist vel;
	
	while (!curr_scan){
		ros::spinOnce();
	}

	if(curr_scan){
		random_planner.updateLaserScan(curr_scan);		
		random_planner.runIteration();
	}

	while(ros::ok()){
		ros::spinOnce();
		//.....**E-STOP DO NOT TOUCH**.......
		eStop.block();
		//...................................


		// Update bumper values in random_planner
		random_planner.bumperLeft = bumperL;
		random_planner.bumperCenter = bumperC;
		random_planner.bumperRight = bumperR;

		// Update laser values in random_planner
		if(curr_scan) {
			random_planner.updateLaserScan(curr_scan);
		}

		// Continuously get random paths
		nav_msgs::Path path = random_planner.getPath();
		vis.publishPath(path, std::chrono::milliseconds(10));
		if (!random_planner.getVelocity(vel) && curr_scan){
			random_planner.updateLaserScan(curr_scan);			
			random_planner.runIteration();
			std::cout << "Getting new plan!" << std::endl;
		}

 		vel_pub.publish(vel);
	}

	return 0;
}
