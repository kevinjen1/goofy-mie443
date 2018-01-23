#include "planner/primitive_planner.hpp"
#include <sensor_msgs/LaserScan.h>
#include <math.h>

#define Pi 3.14159265

namespace goofy{
namespace planner{

bool PrimitivePlanner::getVelocity(geometry_msgs::Twist& vel){
	//todo: this function is a little confusing
	if (_new_plan == true){
		_motion_index = _plan.begin();
		_end_motion_time = std::chrono::steady_clock::now() + std::chrono::milliseconds((*_motion_index).time);
		_vel.linear.x = (*_motion_index).linear_velocity;
		_vel.angular.z = (*_motion_index).angular_velocity;
		_new_plan = false;
		std::cout << "Getting velocity with " << (*_motion_index).time << " milliseconds" << std::endl;
	}

	if (std::chrono::steady_clock::now() < _end_motion_time){
		vel = _vel;
		std::chrono::milliseconds left = std::chrono::duration_cast<std::chrono::milliseconds>(_end_motion_time - std::chrono::steady_clock::now());
		std::cout << "Plan OK -- " << left.count() << " milliseconds left" << std::endl;
	} else {
		_motion_index++;
		if (_motion_index == _plan.end()){
			std::chrono::milliseconds left = std::chrono::duration_cast<std::chrono::milliseconds>(_end_motion_time - std::chrono::steady_clock::now());
			std::cout << "Plan has ended" << std::endl;

			//reset plan and reset path
			_plan.clear();
			_path.poses.clear();

			return false;
		} else {
			std::cout << "Current motion has ended" << std::endl;
			_end_motion_time = std::chrono::steady_clock::now() + std::chrono::milliseconds((*_motion_index).time);
			_vel.linear.x = (*_motion_index).linear_velocity;
			_vel.angular.z = (*_motion_index).angular_velocity;
			vel = _vel;
		}
	}
	return true;
}

void RandomPlanner::runIteration(){
	bool success = false;
	int plan_index = rand() % _primitives.getLength();
	while (success == false){
		success = checkPath(_primitives.getPath(plan_index, common::BASE));
	}
	//add the primitive motion to the plan
	_plan.push_back(_primitives.getMotion(plan_index));

	//add the associated path to the path
	nav_msgs::Path cur_path = _primitives.getPath(plan_index, common::BASE);
	_path.poses.insert(_path.poses.end(), cur_path.poses.begin(), cur_path.poses.end());

	_new_plan = true;
}

bool PrimitivePlanner::checkObstacle(const sensor_msgs::LaserScan::ConstPtr& msg, float x_pos, float y_pos)
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

}
}

