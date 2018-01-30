#include "planner/primitive_planner.hpp"
#include <sensor_msgs/LaserScan.h>
#include <math.h>
#include <stdio.h>
using namespace std;

namespace goofy{
namespace planner{

bool PrimitivePlanner::getVelocity(geometry_msgs::Twist& vel){
	//todo: this function is a little confusing
	if (_plan.empty()) {
		return false;
	}
	else {
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
			// std::cout << "Plan OK -- " << left.count() << " milliseconds left" << std::endl;
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
}

void RandomPlanner::runIteration(){
	bool success = false;
	int plan_index = rand() % _primitives.getLength();
	while (success == false){
		plan_index = rand() % _primitives.getLength();
		success = checkPath(_primitives.getPath(plan_index, common::BASE));		
	}
	
	/*  Using the extra bin width so this isn't needed, 
		but if we decide to do the shorten path method, this is what we would use:

		//see how much to shorten paths to avoid collisions
		double shortTime = shortenPathTimeTo(_primitives.getPath(plan_index, common::BASE));
		_primitives.getPlan().time = int (shortTime);
	*/

	// add the primitive motion to the plan
	_plan.push_back(_primitives.getMotion(plan_index));

	//add the associated path to the path
	nav_msgs::Path cur_path = _primitives.getPath(plan_index, common::BASE);
	_path.poses.insert(_path.poses.end(), cur_path.poses.begin(), cur_path.poses.end());

	_new_plan = true;
}

float RandomPlanner::shortenPathTimeTo(nav_msgs::Path path){
	/* Calculate the time to shorten the path so that it will end the path outside of the area of the robot
	*/
	int i = 0;
	double end_position = (path.poses.end().pose.position.x, path.poses.end().pose.position.y);
	int pose_points = path.poses.size();
	for (i = path.poses.size()-1; i >=0; i--){
		double curr_position = (path.poses[i].pose.position.x, path.poses[i].pose.position.y);
		double dist = sqrt(pow(curr_position[0],2) + pow(curr_position[1],2));
		if (dist > robotRadius){
			break;
		}
	}
	return _primitives.getPlan().time*(path.poses.size() - i)/path.poses.size();
}

bool PrimitivePlanner::checkObstacle(float x_pos, float y_pos, float scan_angle){
	/* Given (x_pos,y_pos) and a scan angle, determine whether there is an obstacle there. 
	return true if there is an obstacle. Return false if there is no obstacle
	*/	
	
	// Step 1 - take the (x_pos,y_pos) and calculate the angle and tangent.
	float angle = atan2(y_pos,x_pos);
	float tangent = sqrt((y_pos)*(y_pos) + (x_pos)*(x_pos));
	bool obstacle = false;
	
	// Step 2 - Check if the position is in your view.
	if (_scan->angle_max < angle || _scan->angle_min > angle){		
		return obstacle;
	}
	else if (_scan->range_min > tangent || _scan->range_max < tangent){	
		return obstacle;
	}
	else {
		// Step 3 - Check position for obstacle.
		int index = (angle - (_scan->angle_min))/(_scan->angle_increment);
		int laserSize = (_scan->angle_max -_scan->angle_min)/_scan->angle_increment;
		int scan_width = (scan_angle)/_scan->angle_increment;
	
		for(int i = index-scan_width; i <= index+scan_width; i++){
			if(0 <= i <= laserSize){
				if(tangent > _scan->ranges[i]){
					obstacle = true;	
				}
			}
		}
		return obstacle;
	}
}

bool PrimitivePlanner::checkPath(nav_msgs::Path path){
	/*  Goes through each of the points in a path
	Uses checkObstacle to see if there is an obstacle along the path 
	Outputs bool indicating if an object lies along the path
	*/

	int hit_points = 0;
	int pose_points = path.poses.size();
	for (int i = 0; i < pose_points; i++){
		hit_points += checkObstacle(path.poses[i].pose.position.x, path.poses[i].pose.position.y, 2);
	}

	return (hit_points == 0);
}

}
}

