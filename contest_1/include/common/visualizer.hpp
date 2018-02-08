#pragma once

#include<ros/ros.h>
#include<nav_msgs/Path.h>
#include<geometry_msgs/Twist.h>
#include<visualization_msgs/Marker.h>

#include<chrono>

namespace goofy {
namespace common {

class Visualizer{
public:
	Visualizer(){
		_path_publisher = _nh.advertise<nav_msgs::Path>("/debug/path_publisher",1);
		_cmd_vel_pub = _nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);
		_point_publisher = _nh.advertise<visualization_msgs::Marker>("path_error_point", 1);
		_laser_publisher = _nh.advertise<visualization_msgs::Marker>("laser_error_point", 1);

		template_marker.type = visualization_msgs::Marker::SPHERE;
		template_marker.action = visualization_msgs::Marker::ADD;
		template_marker.scale.x = 0.1;
		template_marker.scale.y= 0.1;
		template_marker.scale.z = 0.1;
		template_marker.color.a = 1.0;
		template_marker.color.r = 1.0;
		template_marker.color.g = 0.0;
		template_marker.color.b = 0.0;
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

	void publishErrorPoint(float x, float y, std::chrono::milliseconds time){
		std::chrono::steady_clock::time_point enter = std::chrono::steady_clock::now();
		visualization_msgs::Marker marker = template_marker;
		marker.pose.position.x = x;
		marker.pose.position.y = y;
		marker.pose.position.z = 0;
		marker.color.g = 1.0;
		marker.header.frame_id = "camera_depth_frame";
		while ((std::chrono::steady_clock::now() - enter) < time){
			_point_publisher.publish(marker);
			ros::spinOnce();
		}
	}

	void publishLaserPoint(float range, float angle, std::chrono::milliseconds time){
		std::chrono::steady_clock::time_point enter = std::chrono::steady_clock::now();
		visualization_msgs::Marker marker = template_marker;
		marker.pose.position.x = range*cos(angle);
		marker.pose.position.y = range*sin(angle);
		marker.pose.position.z = 0;
		marker.header.frame_id = "camera_depth_frame";
		while ((std::chrono::steady_clock::now() - enter) < time){
			_laser_publisher.publish(marker);
			ros::spinOnce();
		}
	}






private:
	ros::NodeHandle _nh;
	ros::Publisher _path_publisher;
	ros::Publisher _cmd_vel_pub;
	ros::Publisher _point_publisher;
	ros::Publisher _laser_publisher;

	visualization_msgs::Marker template_marker;
};

}
}
