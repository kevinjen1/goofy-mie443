#pragma once

#include<ros/ros.h>
#include<nav_msgs/Path.h>

namespace goofy {
namespace common {

class Visualizer{
public:
	Visualizer(){
		_path_publisher = _nh.advertise<nav_msgs::Path>("/debug/path_publisher",1);
	}

	void publishPath(nav_msgs::Path path){
		_path_publisher.publish(path);
	}

private:
	ros::NodeHandle _nh;
	ros::Publisher _path_publisher;
};

}
}
