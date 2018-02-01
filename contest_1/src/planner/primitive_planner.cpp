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
		common::BasicMotion shortened = _primitives.getMotion(plan_index);
		double temp = shortened.time*shortTime;
		shortened.time = int(temp);

		// add the primitive motion to the plan
		_plan.push_back(shortened);
	*/

	// add the primitive motion to the plan
	_plan.push_back(_primitives.getMotion(plan_index));

	//add the associated path to the path
	nav_msgs::Path cur_path = _primitives.getPath(plan_index, common::BASE);
	_path.poses.insert(_path.poses.end(), cur_path.poses.begin(), cur_path.poses.end());

	_new_plan = true;
}


void WeightedPlanner::runIteration(){
	/* Need implementing 
        Does all the same path overhead as RandomPlanner::runIteration above
        Uses PrimitivePlanner::checkPath to see if each path has an obstacle
        Split up the obstacle-free paths into their max/min POV angle bounds
        For each one, find bins, and sum up laser distance readings
        Note: Need to handle too close/far. Ignore if NaN, use initial default - what do if all NaN?
        Pick the path with the max value (furthest from an obstacle)
    */

    int max_path = -1;
    float max_outcome = 0;
	for (int i = 0; i <= _primitives.getLength(); i++) {
		float outcome = checkPath(_primitives.getPath(i, common::BASE));       
        if (outcome == -2) {
            // do something.. right now nothing in mind
        }
        else if (outcome >= max_outcome){
            max_path = i;
        }	
    }

    // Need some handle for if no paths are good
	
	/*  Using the extra bin width so this isn't needed, 
		but if we decide to do the shorten path method, this is what we would use:
	
		//see how much to shorten paths to avoid collisions
		double shortTime = shortenPathTimeTo(_primitives.getPath(plan_index, common::BASE));
		common::BasicMotion shortened = _primitives.getMotion(plan_index);
		double temp = shortened.time*shortTime;
		shortened.time = int(temp);

		// add the primitive motion to the plan
		_plan.push_back(shortened);
	*/

	// add the primitive motion to the plan
	_plan.push_back(_primitives.getMotion(max_path));

	//add the associated path to the path
	nav_msgs::Path cur_path = _primitives.getPath(max_path, common::BASE);
	_path.poses.insert(_path.poses.end(), cur_path.poses.begin(), cur_path.poses.end());

	_new_plan = true;
}

double PrimitivePlanner::shortenPathTimeTo(nav_msgs::Path path){
	/* Calculate the time ratio to shorten the path so that it will end the path outside of the area of the robot
	*/
	int i = 0;
	int pose_points = path.poses.size();
	double end_position[2] = {path.poses[pose_points-1].pose.position.x, path.poses[pose_points-1].pose.position.y};
	for (i = path.poses.size()-1; i >=0; i--){
		double curr_position[2] = {path.poses[i].pose.position.x, path.poses[i].pose.position.y};
		double dist = sqrt(pow(curr_position[0]-end_position[0],2) + pow(curr_position[1]-end_position[1],2));
		if (dist > robotRadius){
			break;
		}
	}
	return (path.poses.size() - i)/path.poses.size();
}

float PrimitivePlanner::scanWidthAngle(nav_msgs::Path path, float x, float y){
	/*  This is a helper function for checkPath, to be passed to checkObstacle
		The idea is to find the angle between the robot's current position, and the edges of the robot at the point being checked
		This is to account for the non-point-mass nature of the robot, for collision avoidance
	*/	
	double curr_position[2] = {path.poses[0].pose.position.x, path.poses[0].pose.position.y};
	double dist = sqrt(pow(curr_position[0]-x,2) + pow(curr_position[1]-y,2));
	return atan2(dist, robotRadius);
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
		float x = path.poses[i].pose.position.x;
		float y = path.poses[i].pose.position.y;
		float scan_angle = scanWidthAngle(path, x, y);
		//float scan_angle = 2;
		hit_points += checkObstacle(x, y, scan_angle);
	}

	return (hit_points == 0);
}

float WeightedPlanner::scanWidthAngle(float curr_x, float curr_y, float x, float y){
	/*  This is a helper function for checkPath, to be passed to checkObstacle
		The idea is to find the angle between the robot's current position, and the point being checked
		This is to help tally up the min/max angle bounds for diving paths into laser bin ranges
	*/	
	double curr_position[2] = {curr_x, curr_y};
	return atan2(curr_position[1]-y, curr_position[0]-x);
}

float WeightedPlanner::getDistance(float max_angle, float min_angle){
	/* 
        Returns:
            -1 if out of FOV
            -2 if all values are NaN
            Otherwise returns sum of all buckets in range
	*/	
	
	// Step 2 - Check if the position is in your view.
	if (_scan->angle_max < max_angle || _scan->angle_min > min_angle){		
		return -1;
	}

    int numBins = (_scan->angle_max -_scan->angle_min)/_scan->angle_increment;
    int min_bucket = ((_scan->angle_max -_scan->angle_min)/2 - min_angle)/_scan->angle_increment;
    int max_bucket = (max_angle - (_scan->angle_max -_scan->angle_min)/2)/_scan->angle_increment;
    float bucket_sum = -2;
    //int scan_width = (scan_angle)/_scan->angle_increment;
    for(int i = min_bucket; i <= max_bucket; i++){
    	if(!isnan(_scan->ranges[i])){
			bucket_sum += _scan->ranges[i];
		}
	}
	return bucket_sum;
}

float WeightedPlanner::checkPath(nav_msgs::Path path){
	/*  Needs implementing 
        Returns:
            -1 if invalid path (out of FOV or obstacles hit)
            -2 if all readings are NaN
            Otherwise returns sum of all buckets in range
    */

    float min_angle = 0;
    float max_angle = 50;
    int pose_points = path.poses.size();

	for (int i = 1; i < pose_points; i++){
		float x = path.poses[i].pose.position.x;
		float y = path.poses[i].pose.position.y;
		float scan_angle = scanWidthAngle(path.poses[0].pose.position.x, path.poses[0].pose.position.y, x, y);

        // Check for obstacle collisions / out of scan range, return invalid path
        if (checkObstacle(x, y, scan_angle)){
            return -1;
        }

        // Get the mix/max angles of points along the path
        if (scan_angle > max_angle) {max_angle = scan_angle;}
        if (scan_angle < min_angle) {min_angle = scan_angle;}
	}

	return getDistance(max_angle, min_angle);
}

}
}

