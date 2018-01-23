#include "planner/primitive_planner.hpp"
#include <sensor_msgs/LaserScan.h>
#include <math.h>

#define Pi 3.14159265

bool checkObstacle(const sensor_msgs::LaserScan::ConstPtr& msg, float x_pos, float y_pos)
{
	// Step 1 - take the (x_pos,y_pos) and calculate the angle and tangent
	float angle = atan2(y_pos,x_pos);
	float tangent = sqrt((y_pos)*(y_pos) + (x_pos)*(x_pos));
	
	// Step 2 - Check if the position is in your view.
	if(msg->angle_max < angle || msg->angle_min > angle){
		return 0;
	}
	else if(msg->range_min > tangent || msg->range_max < tangent){
		return 0;
	}

	// Step 3 - Check position for obstacle.
	int index = (angle - (msg->angle_min))/(msg->angle_increment);
	if(tangent > msg->ranges[index]){
		return 1;	
	}
	else {
		return 0;
	}
}

