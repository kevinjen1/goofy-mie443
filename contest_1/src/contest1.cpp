#include <ros/console.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <eStop.h>

#include <stdio.h>
#include <cmath>
#include"mapper/mapper.hpp"
#include"planner/primitive_planner.hpp"
#include"common/common.hpp"
#include <tf/transform_listener.h>
using namespace std;
using namespace goofy;

bool bumperL = 0, bumperC = 0, bumperR = 0;
double lRange = 10;
int lSize = 0, lOffset = 0, dAngle = 5;
sensor_msgs::LaserScan::Ptr curr_scan;
geometry_msgs::Pose2D nextPoint;
geometry_msgs::Pose2D currPose;

void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg){
	// This callback updates the left right and center bumper states
	if(msg->bumper == 0 && msg->state == 1){
		bumperL = !bumperL;
	}
	else if(msg->bumper == 1 && msg->state == 1){
		bumperC = !bumperC;
	}
	else if(msg->bumper == 2 && msg->state == 1){
		bumperR = !bumperR;
	}
	else if(msg->state == 0){
		bumperL = 0;
		bumperC = 0;
		bumperR = 0;
	}
}

void getNextPoint(geometry_msgs::Pose2D nextPose){
	nextPoint = nextPose;
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){

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

	//recast this as a non-const pointer
	curr_scan = boost::make_shared<sensor_msgs::LaserScan>(*(msg.get()));
	return;
}

//void callbackOdom(const nav_msgs::Odometry odom)
//{
//	// Find robot 2D pose from odometry
//	// Position (x,y)
//	currPose.x = odom.pose.pose.position.x;
//	currPose.y = odom.pose.pose.position.y;
//
//	currPose.theta = goofy::common::quat2yaw(odom.pose.pose.orientation);
//}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "image_listener");
	ros::NodeHandle nh;
	teleController eStop;

	ros::Subscriber bumper_sub = nh.subscribe("mobile_base/events/bumper", 10, &bumperCallback);
	ros::Subscriber laser_sub = nh.subscribe("scan", 10, &laserCallback);

	ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);

	ros::Subscriber next_coord_sub = nh.subscribe("goofCoord", 1, &getNextPoint);
	//ros::Subscriber sub_odom = nh.subscribe("odom", 1, &callbackOdom);

	tf::TransformListener listener;

	//Setup Robot
	common::RobotModel robot(0.5,0.5,0.5);

	//Setup Primitives
	common::BasicMotion straight{0.2, 0, 4000};
	//common::BasicMotion mild_right{0.15, -0.1, 4000};
	//common::BasicMotion mild_left{0.15, 0.1, 4000};
	common::BasicMotion turn_left{0.15,0.15, 4000};
	common::BasicMotion turn_right{0.15,-0.15, 4000};
	common::BasicMotion turn_sharp_left{0.15, 0.3, 3000};
	common::BasicMotion turn_sharp_right{0.15, -0.3, 3000};
	common::BasicMotion on_spot_right{0, -0.3, 2000};
	common::BasicMotion on_spot_left{0, 0.3, 2000};

	planner::MotionList motions;
	motions.push_back(straight);
	motions.push_back(turn_left);
	motions.push_back(turn_right);
	motions.push_back(turn_sharp_left);
	motions.push_back(turn_sharp_right);
	motions.push_back(on_spot_right);
	motions.push_back(on_spot_left);

	//Setup Planner
	planner::PrimitiveRepresentation primitives(robot, motions);
	//planner::WeightedPlanner random_planner(primitives);
	planner::HeuristicPlanner random_planner(primitives);
	//planner::PrimitivePlanner random_planner(primitives);
	common::Visualizer vis;

	geometry_msgs::Twist stop;
	stop.angular.z = 0;
	stop.linear.x = 0;
	geometry_msgs::Twist vel = stop;


	while (!curr_scan){
		ros::spinOnce();
	}

	if(curr_scan){
		common::filterLaserScan(curr_scan, 50);
		random_planner.updateLaserScan(curr_scan);
		random_planner.addSpin();
	}

	while(ros::ok()){
		ros::spinOnce();
		//.....**E-STOP DO NOT TOUCH**.......
		eStop.block();
		//...................................

		tf::StampedTransform transform;
		try{
		  listener.lookupTransform("/map", "/base_link",
								   ros::Time(0), transform);
		  //ROS_INFO_STREAM("Got the transform");
		}
		catch (tf::TransformException ex){
		  ROS_ERROR("%s",ex.what());
		  ros::Duration(1.0).sleep();
		}
		//ROS_INFO_STREAM("transform pos: "<< transform.getOrigin().x() << ", " << transform.getOrigin().y());

		currPose.x = transform.getOrigin().x();
		currPose.y = transform.getOrigin().y();

		geometry_msgs::Quaternion quat;
		quat.x = transform.getRotation().x();
		quat.y = transform.getRotation().y();
		quat.z = transform.getRotation().z();
		quat.w = transform.getRotation().w();

		currPose.theta = common::quat2yaw(quat);


		// Update bumper values in random_planner
		random_planner.bumperLeft = bumperL;
		random_planner.bumperCenter = bumperC;
		random_planner.bumperRight = bumperR;
		random_planner.nextPosition = nextPoint;
		random_planner.current_pose = currPose;

		// Update laser values in random_planner
		if(curr_scan) {
			common::filterLaserScan(curr_scan, 50);
			random_planner.updateLaserScan(curr_scan);
		}

		// Continuously get random paths
		nav_msgs::Path path = random_planner.getPath();
		vis.publishPath(path, std::chrono::milliseconds(10));
		if (!random_planner.getVelocity(vel) && curr_scan){
			vel_pub.publish(stop); //need to publish a zero velocity here!
			common::filterLaserScan(curr_scan, 50);
			random_planner.updateLaserScan(curr_scan);
			random_planner.runIteration();
			std::cout << "Getting new plan!" << std::endl;
			random_planner.getVelocity(vel);
		}

 		vel_pub.publish(vel);
	}

	return 0;
}
