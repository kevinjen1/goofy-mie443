#include <ros/console.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/LaserScan.h>
#include <eStop.h>

#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

#include <stdio.h>
#include <cmath>

#include <nav_msgs/OccupancyGrid.h>

using namespace std;

int laser_mode = 0;
// Mode for testing lasers:
//	0 = no laser used, for the odometry and bumper test
//	1 = laser used in laserCallBack function part 1 (slide 41)
//	2 = laser used in laserCallBack function part 2 (slide 41)

double angular;
double linear;

double posX, posY, yaw;
double pi = 3.1416;

bool bumperLeft = 0, bumperCenter = 0, bumperRight = 0;

double laserRange = 10;
int laserSize = 0, laserOffset = 0, desiredAngle = 5;

void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg){
	if(msg.bumper == 0)
		bumperLeft = !bumperLeft;
	else if(msg.bumper == 1)
		bumperCenter = !bumperCenter;
	else if(msg.bumper == 2)
		bumperRight = !bumperRight;
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){

	laserSize = (msg->angle_max -msg->angle_min)/msg->angle_increment;
	laserOffset = desiredAngle*pi/(180*msg->angle_increment);
	laserRange = 11;
	
	if (laser_mode == 2)
	{
		if (desiredAngle*pi/180 < msg->angle_max && -desiredAngle*pi/180 > msg->angle_min){
			for (int i = laserSize/2 - laserOffset; i < laserSize/2 + laserOffset; i++){
				if (laserRange > msg->ranges[i])
					laserRange = msg->ranges[i];
			}
		}
		else{
			for (int i = 0; i < laserSize; i++){
				if (laserRange > msg->ranges[i])
					laserRange = msg->ranges[i];
			}
		}

		if (laserRange == 11)
			laserRange = 0;
	}
	else if (laser_mode == 1)
		ROS_INFO("Size of laser scan array: %i and size of offset: %i", laserSize, laserOffset);
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
	posX = msg->pose.pose.position.x;
	posY = msg->pose.pose.position.y;
	yaw = tf::getYaw(msg->pose.pose.orientation);

	if (!laser_mode)
		ROS_INFO("Postion: (%f, %f) Orientation: %f rad or %f degrees.", posX, posY, yaw, yaw*180/pi);
}

void occupancyCallback(const nav_msgs::OccupancyGrid& msg){
	ROS_INFO("Width: %i, Height: %i, Resolution: %f, Origin: (%f, %f), Random Map: %d", msg.info.width, msg.info.height, msg.info.resolution, msg.info.origin.position.x, msg.info.origin.position.y, msg.data[msg.info.width*msg.info.height-1]);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "image_listener");
	ros::NodeHandle nh;
	teleController eStop;

	ros::Subscriber bumper_sub = nh.subscribe("mobile_base/events/bumper", 10, &bumperCallback);

	// For laser, they had two.. should be the top one, but kept the 2nd one (commented out) in case
	ros::Subscriber laser_sub = nh.subscribe("scan", 10, &laserCallback);
	//image_transport::ImageTransport it(nh);
	//image_transport::Subscriber depth_sub = it.subscribe("camera/depth_registered/image_raw", 1, &laserCallback);

	ros::Subscriber odom = nh.subscribe("odom", 1, &odomCallback);
	ros::Subscriber grid_occ = nh.subscribe("map", 10, &occupancyCallback);

	ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);

	angular = 0.0;
	linear = 0.0;
	geometry_msgs::Twist vel;

	while(ros::ok()){
		ros::spinOnce();
		//.....**E-STOP DO NOT TOUCH**.......
		eStop.block();
		//...................................

		if (!laser_mode)
			laserRange = 0.8;	// If we aren't using the lasers, this ignore lasers in if conditions below

		ROS_INFO("Postion: (%f, %f) Orientation: %f degrees Range: %f", posX, posY, yaw*180/pi, laserRange);
		if (posX < 0.5 && yaw < pi/2 && !bumperLeft && !bumperCenter && !bumperRight && laserRange > 0.7){
			angular = 0.0;
			linear = 0.2;
		}
		else if (posX > 0.5 && yaw < pi/2 && !bumperLeft && !bumperCenter && !bumperRight && laserRange > 0.5){
			angular = pi/6;
			linear = 0.0;
		}
		else if (laserRange > 1.0 && !bumperLeft && !bumperCenter && !bumperRight){
			if (yaw < 17*pi/36 || posX > 0.6){
				angular = pi/12;
				linear = 0.1;
			}
			else if (yaw < 19*pi/36 || posX > 0.4){
				angular = -pi/12;
				linear = 0.1;
			}
			else{
				angular = 0.0;
				linear = 0.0;
			}
		}
		else{
			angular = 0.0;
			linear = 0.0;
		}

  		vel.angular.z = angular;
  		vel.linear.x = linear;

  		vel_pub.publish(vel);
	}

	return 0;
}
