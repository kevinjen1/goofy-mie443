#pragma once

#include<cmath>

#include<nav_msgs/Path.h>
#include<geometry_msgs/PoseStamped.h>
#include<geometry_msgs/Quaternion.h>
#include<sensor_msgs/LaserScan.h>

#include<tf/LinearMath/Quaternion.h>
#include<tf/LinearMath/Matrix3x3.h>

namespace goofy{
namespace common{

const std::string BASE = "base_link";
const std::string ODOM = "odom";
const std::string WORLD = "world";

typedef std::vector<geometry_msgs::PoseStamped> PoseArray;

struct BasicMotion{
	double linear_velocity; //in meters per second
	double angular_velocity; //in meters per second
	int time; //in milliseconds
};

class RobotModel{
public:
	RobotModel(double left_rad, double right_rad, double wheelbase):
		_left_rad(left_rad),
		_right_rad(right_rad),
		_wheelbase(wheelbase){}

	/**
	 * Linear Velocity in m/s
	 * Angular Velocity in rad/s
	 * Time in s
	 * Density in m
	 */
	nav_msgs::Path simulatePath(double lin_vel, double ang_vel, double time, double density);
	nav_msgs::Path simulatePath(BasicMotion motion, double density);

private:
	double _left_rad;
	double _right_rad;
	double _wheelbase;
};

geometry_msgs::Quaternion yaw2quat(double yaw);
double quat2yaw(geometry_msgs::Quaternion quaternion);
void filterLaserScan(sensor_msgs::LaserScan& scan, int window);

}
}
