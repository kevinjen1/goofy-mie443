#include "planner/primitive_planner.hpp"
#include <thread>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose2D.h>
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
			_end_motion_time = std::chrono::steady_clock::now() + std::chrono::milliseconds(int((*_motion_index).time * 0.7));
			_vel.linear.x = (*_motion_index).linear_velocity;
			_vel.angular.z = (*_motion_index).angular_velocity;
			_new_plan = false;
			//std::cout << "Getting velocity with " << (*_motion_index).time << " milliseconds" << std::endl;
		}
		
		if (std::chrono::steady_clock::now() < _end_motion_time){
			// if the robot is not at the end of the path, but detects an obstacle, stop.
			// std::cout << "Obstacle index:"<<ifObstacle()<<std::endl;			
			if (ifObstacle() == 0) {
				// Left Bumper pushed in - turn right
				_vel.linear.x = 0;
				_vel.linear.y = 0;

				std::cout << "Left Bump" << std::endl;

				_plan.clear();
				_path.poses.clear();
				_new_plan = true;
				common::BasicMotion back{-0.1, 0, 1000};
				_plan.push_back(back);
				_plan.push_back(_primitives.getMotion(3));
				return true;
			}
			else if (ifObstacle() == 1) {
				// Center Bumper pushed in - move back
				_vel.linear.x = 0;
				_vel.linear.y = 0;

				std::cout << "Center Bump" << std::endl;

				_plan.clear();
				_path.poses.clear();
				_new_plan = true;
				common::BasicMotion back{-0.1, 0, 1000};
				_plan.push_back(back);
				_plan.push_back(_primitives.getMotion(3));
				return true;
			}
			else if (ifObstacle() == 2) {
				// Right Bumper pushed in - move left
				_vel.linear.x = 0;
				_vel.linear.y = 0;

				std::cout << "Right Bump" << std::endl;

				_plan.clear();
				_path.poses.clear();
				_new_plan = true;
				common::BasicMotion back{-0.1, 0, 1000};
				_plan.push_back(back);
				_plan.push_back(_primitives.getMotion(4));
				return true;
			}
			else if (ifObstacle() == 3) {
				// scanned obstacle 
				_vel.linear.x = 0;
				_vel.linear.y = 0;

				std::cout << "Obstacle within 0.5 m" << std::endl;

				_plan.clear();
				_path.poses.clear();
				_new_plan = true;
				_plan.push_back(_primitives.getMotion(3));
				return true;
			}
			/*else if (ifObstacle() == 4) {
				// scanned obstacle to the right
				_vel.linear.x = 0;
				_vel.linear.y = 0;

				std::cout << "Obstacle within 0.5 m on right!" << std::endl;

				_plan.clear();
				_path.poses.clear();
				_new_plan = true;
				_plan.push_back(_primitives.getMotion(4));
				return true;
			}*/						
			vel = _vel;
			std::chrono::milliseconds left = std::chrono::duration_cast<std::chrono::milliseconds>(_end_motion_time - std::chrono::steady_clock::now());
			// std::cout << "Plan OK -- " << left.count() << " milliseconds left" << std::endl;
		} else {
			_motion_index++;
			if (_motion_index == _plan.end()){

				_vel.linear.x = 0;
				_vel.linear.y = 0;

				std::chrono::milliseconds left = std::chrono::duration_cast<std::chrono::milliseconds>(_end_motion_time - std::chrono::steady_clock::now());
				//std::cout << "Plan has ended" << std::endl;

				//reset plan and reset path
				_plan.clear();
				_path.poses.clear();

				return false;
			} else {
				//std::cout << "Current motion has ended" << std::endl;
				_end_motion_time = std::chrono::steady_clock::now() + std::chrono::milliseconds((*_motion_index).time);
				_vel.linear.x = (*_motion_index).linear_velocity;
				_vel.angular.z = (*_motion_index).angular_velocity;
				vel = _vel;
			}
		}
		return true;
	}
}

void PrimitivePlanner::runIteration(){
	bool success = false;
	int plan_index = -1;
	while (success == false){
		plan_index++;
		success = checkPath(_primitives.getPath(plan_index, common::BASE));
		std::cout << "Checked path number: " << plan_index << "and got " << success << std::endl;
		_vis.publishPath(_primitives.getPath(plan_index, common::BASE), std::chrono::milliseconds(500));		
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


void HeuristicPlanner::runIteration(){
    // This implementation is the same as RandomPlanner	(peruses around, avoiding obstacles
    // Differences being:
    //      1) Whenever a destination point arrives, at the beginning of next turn rotate to face it
    //      2) Sort each valid (success==true) path by distance to end point in increasing order

	// If already close to the target position, or there is no target position, randomly navigate
	int redZone = 0.3;
	if (sqrt(pow(nextPosition.x,2) + pow(nextPosition.y,2)) < redZone){
		PrimitivePlanner::runIteration();
		return;
	}

	// If a new path comes in, turn to face it
	if ((currentTargetPosition.x != nextPosition.x) || (currentTargetPosition.y != nextPosition.y)){
		currentTargetPosition = nextPosition;
		//turn to face it
		float target_angle = atan2(nextPosition.y, nextPosition.x);
		if (target_angle < Pi){
			common::BasicMotion on_spot_aim{0, 0.3, (int)(1000*target_angle/0.3)};
			_plan.push_back(on_spot_aim);
			_new_plan = true;
		} else {
			common::BasicMotion on_spot_aim{0, 0.3, (int)(1000*target_angle/0.3)};
			_plan.push_back(on_spot_aim);
			_new_plan = true;
		}
		return;
	}

    int planned_path = -1;
	//int num_on_spots = 2;
	int num_on_spots = getNumberOnSpots();
    pathOptions optionsArray[_primitives.getLength()-num_on_spots];
    for (int plan_index = 0; plan_index < _primitives.getLength()-num_on_spots-1; plan_index++) {
		bool success = checkPath(_primitives.getPath(plan_index, common::BASE));

        nav_msgs::Path temp_path = _primitives.getPath(plan_index, common::BASE);
        float x = temp_path.poses[temp_path.poses.size()-1].pose.position.x;
        float y = temp_path.poses[temp_path.poses.size()-1].pose.position.y;

        // Expecting next_position struct style with .x and .y fields
        float euclid_dist = sqrt(pow(nextPosition.x-x,2) + pow(nextPosition.y-y,2));
        //float euclid_dist = 0;  // while I wait for input

        optionsArray[plan_index] = {plan_index, success, euclid_dist};		
	}

    // Sort the potential paths by euclidean distance (increasing order) from the next_position
    std::vector<pathOptions> optionsVector (optionsArray, optionsArray + _primitives.getLength()-1);
    std::sort (optionsVector.begin(), optionsVector.end(), boolComparison);

    for (int i = 0; i < _primitives.getLength()-1; i++) {
        if (optionsVector[i].valid){
            planned_path = optionsVector[i].index;
            //std::cout << "Checked path number: " << planned_path << std::endl;
    		_vis.publishPath(_primitives.getPath(planned_path, common::BASE), std::chrono::milliseconds(500));
            break;
        }
    }

    // If none of the paths are valid, spin on the spot
    // Still want to add some functionality for choosing to turn left or right, or custom amounts
    if (planned_path < 0){
        planned_path = _primitives.getLength()-1;
	}

	// add the primitive motion to the plan
	_plan.push_back(_primitives.getMotion(planned_path));

	//add the associated path to the path
	nav_msgs::Path cur_path = _primitives.getPath(planned_path, common::BASE);
	_path.poses.insert(_path.poses.end(), cur_path.poses.begin(), cur_path.poses.end());

	_new_plan = true;
}

int HeuristicPlanner::getNumberOnSpots(){
	/*  Count the number of paths that have the same startpoint as endpoint
		The idea behind this is to leave all turn-only moves as emergency (at the end of the path list), 
		and only evaluate moves that require moving forward
	*/
	int count = 0;
	for (int i = 0; i < _primitives.getLength(); i++){
		nav_msgs::Path temp_path = _primitives.getPath(i, common::BASE);
		float start_position_x = temp_path.poses[0].pose.position.x;
		float start_position_y = temp_path.poses[0].pose.position.y;
		float end_position_x = temp_path.poses[temp_path.poses.size()-1].pose.position.x;
		float end_position_y = temp_path.poses[temp_path.poses.size()-1].pose.position.y;
		if ((start_position_x == end_position_x) && (start_position_y == end_position_y)){
			count++;
		}
	}
	return count;
}

bool HeuristicPlanner::boolComparison(pathOptions i, pathOptions j){
    return (i.euclid_dist < j.euclid_dist);
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

//	int backupPathIndex;
    int max_path = -1;
    float max_outcome = 0;
	for (int i = 0; i < _primitives.getLength()-1; i++) {
		// Hacked way to see if it is on_spot		
		//if (_primitives.getPath(i, common::BASE).linear_velocity == 0){
//		nav_msgs::Path tempPath = _primitives.getPath(i, common::BASE);
//		if (tempPath.poses[0].pose.position == tempPath.poses[tempPath.poses.size()-1].pose.position){
//			backupPathIndex = i;
//			continue;
//		}
		float outcome = checkPath(_primitives.getPath(i, common::BASE));    
        //std::cout << "Checked path number: " << i << std::endl;   
		_vis.publishPath(_primitives.getPath(i, common::BASE), std::chrono::milliseconds(1000));
        if (outcome >= max_outcome){
            max_path = i;
        }	
    }

    // If no paths are good, select spin on the spot
	if (max_path < 0){
		max_path = _primitives.getLength()-1;
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
	double dist = sqrt(pow(curr_position[0]-x,2) + pow(curr_position[1]-y,2)) + 0.1;
	return atan2(robotRadius, dist);
}

bool PrimitivePlanner::checkObstacle(float x_pos, float y_pos, float scan_angle){
	/* Given (x_pos,y_pos) and a scan angle, determine whether there is an obstacle there. 
		return true if there is an obstacle. Return false if there is no obstacle
	*/	
	
	// take the (x_pos,y_pos) and calculate the angle and tangent.
	float angle = atan2(y_pos,x_pos);
	float tangent = sqrt((y_pos)*(y_pos) + (x_pos)*(x_pos));
	bool obstacle = false;
	
	if (_scan->angle_max < angle || _scan->angle_min > angle){		
		// outside of the viewing angle is not an obstacle				
		return false;
	}
	else if (_scan->range_min > tangent || _scan->range_max < tangent){	
		// anything outside of the range is not an obstacle				
		return false;
	}
	else {
		// Check (x_pos,y_pos) position for obstacle.
		int index = (angle - (_scan->angle_min))/(_scan->angle_increment);
		int laserSize = (_scan->angle_max -_scan->angle_min)/_scan->angle_increment;
		int scan_width = (scan_angle)/_scan->angle_increment;
	
		for(int i = index-scan_width; i <= index+scan_width; i++){
			if(0 <= i <= laserSize){
				if(tangent > _scan->ranges[i] || std::isnan(_scan->ranges[i])){
					return true;	
				}
			}
		}
	}
	return obstacle;
}

int PrimitivePlanner::ifObstacle(){
	/* This function is used as an interrupt.
		At any point, if there is an obstacle, it'll start re planning a path.
		This includes laser scans and bumper.
		It returns: 
			- 0 for left bump
			- 1 for center bump
			- 2 for right bump
			- 3 for scanned obstacle to the left
			- 4 for scanned obstacle to the right
			- 5 for nothing
	*/	

	int laserSize = (_scan->angle_max -_scan->angle_min)/_scan->angle_increment;
	/*int turn_right = 0;
	int turn_left = 0;
	for (int i = 1; i <= laserSize; i++){
		if(_scan->ranges[i] < 0.5) { 
			if(i<= laserSize/2) {
				turn_right+= _scan->ranges[i];
			} 
			else {
				turn_left+= _scan->ranges[i];
			}	
		}
	}
	if(turn_right != 0 && turn_left != 0) {
		if(turn_right >= turn_left) {
			return 3;
		}
		else if(turn_left >= turn_right){
			return 4;
		}
	}*/
	for (int i = 1; i <= laserSize; i++){
		if(_scan->ranges[i] < 0.5) { 
			return 3;
		}
	}
	if(bumperLeft == 1) {
		return 0;
	}
	else if(bumperCenter == 1) {
		return 1;
	}
	else if(bumperRight == 1) {
		return 2;
	}
	else {
		return 4;
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
		if (hit_points == 1) {
			return 0;
		}
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
            -1 if invalid path (out of FOV or obstacles hit)
            Otherwise returns sum of all buckets in range
	*/	
	
	// Check if the position is in your view.
	if (_scan->angle_max < max_angle || _scan->angle_min > min_angle){		
		return -1;
	}

    int min_bucket = (min_angle - _scan->angle_min)/_scan->angle_increment;
    int max_bucket = (max_angle - _scan->angle_min)/_scan->angle_increment;
    float bucket_sum = -1;
    //int scan_width = (scan_angle)/_scan->angle_increment;
    for(int i = min_bucket; i <= max_bucket; i++){
    	if(isnan(_scan->ranges[i])){
			return -1;
		}
		bucket_sum += _scan->ranges[i];
	}
	return bucket_sum;
}

float WeightedPlanner::checkPath(nav_msgs::Path path){
	/*  Needs implementing 
        Returns:
            -1 if invalid path (out of FOV or obstacles hit)
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

