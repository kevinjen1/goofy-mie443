#pragma once

#include<ros/ros.h>
#include<nav_msgs/Path.h>
#include<geometry_msgs/Twist.h>

#include<chrono>

namespace goofy {
namespace common {

class Visualizer{
public:
	Visualizer(){
		_path_publisher = _nh.advertise<nav_msgs::Path>("/debug/path_publisher",1);
		_cmd_vel_pub = _nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);
	}

	void publishPath(nav_msgs::Path path, std::chrono::milliseconds time){
		std::chrono::steady_clock::time_point enter = std::chrono::steady_clock::now();
		while ((std::chrono::steady_clock::now() - enter) < time){
			_path_publisher.publish(path);
			ros::spinOnce();
		}
	}

	void publishVelocity(geometry_msgs::Twist vel, std::chrono::milliseconds time){
		std::chrono::steady_clock::time_point enter = std::chrono::steady_clock::now();
		while ((std::chrono::steady_clock::now() - enter) < time){
			_cmd_vel_pub.publish(vel);
			ros::spinOnce();
		}
	}

private:
	ros::NodeHandle _nh;
	ros::Publisher _path_publisher;
	ros::Publisher _cmd_vel_pub;
};

}
}
