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
			if (_recovery == false){
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
					common::BasicMotion recovery_turn_right{0, -0.5, 2000};
					_plan.push_back(recovery_turn_right);
					_recovery = true;
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
					common::BasicMotion recovery_turn_right{0,  -0.5 + (rand_num > 0.5), 2000};
					_plan.push_back(recovery_turn_right);
					_recovery = true;
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
					common::BasicMotion back{-0.1, 0, 2000};
					_plan.push_back(back);
					common::BasicMotion recovery_turn_left{0, 0.5, 1500};
					_plan.push_back(recovery_turn_left);
					_recovery = true;
					return true;
				}
				else if (ifObstacle() == 3) { //if we are not in a recovery behavior
					// scanned obstacle
					_vel.linear.x = 0;
					_vel.linear.y = 0;

					std::cout << "Obstacle within 0.5m!" << std::endl;

					_plan.clear();
					_path.poses.clear();
					_new_plan = true;
					std::cout << "Radn num:  " << rand_num << std::endl;
					common::BasicMotion recovery_turn_right{0, -0.5 + (rand_num > 0.5) , 500};
					_plan.push_back(recovery_turn_right);
					_recovery = true;
					return true;
				}
			}
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

				_recovery = false;

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

void PrimitivePlanner::addSpin(){
	std::cout << "STARTED 360 SPIN" << std::endl;
	common::BasicMotion rotate{0, 0.5, (int)((2*Pi*2000)/0.5)};
	_plan.push_back(rotate);
	_recovery = true;
	_new_plan = true;
	std::cout << "Exiting spinonce fcn" << std::endl;
}

void PrimitivePlanner::runIteration(){
	bool success = false;
	_recovery = false;
	int plan_index = -1;
	std::cout << "\nTotal paths: " << _primitives.getLength() << std::endl;
	while (success == false){
		plan_index++;
		if (plan_index >= _primitives.getLength()){
		    return;
	    }
		success = checkPath(_primitives.getPath(plan_index, common::BASE));
		std::cout << "Checked path number: " << plan_index << " and got  " << success << std::endl;
		_vis.publishPath(_primitives.getPath(plan_index, common::BASE), std::chrono::milliseconds(5));
		_vis.publishErrorPoint(0, 0, std::chrono::milliseconds(5));
		_vis.publishLaserPoint(0, 0, std::chrono::milliseconds(5));
		
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

	// Convert global poses to be in local reference frame

	// If a new path comes in, turn to face it
	if ((currentTargetPosition.x != nextPosition.x) || (currentTargetPosition.y != nextPosition.y)){
		std::cout << "\n=======================" << endl;
		std::cout << "[HP->runIter] NEW POINT" << endl;
		std::cout << "=======================" << endl;
		currentTargetPosition = nextPosition;
		randomFlag = false;
		getLocalTargetPosition();
		//turn to face it
		float target_angle = atan2(local_target_pose.y, local_target_pose.x);
		std::cout << "[HP->runIter] Target angle to the new path: " << target_angle << std::endl;
		/*
		if (target_angle < Pi){
			common::BasicMotion on_spot_aim{0, 0.3, (int)(1000*target_angle/0.3)};
			_plan.push_back(on_spot_aim);
			_new_plan = true;
			std::cout << "[HP->runIter] New global target point detected at (" << currentTargetPosition.x << ", " << currentTargetPosition.y << ")" << std::endl; 
			std::cout << "[HP->runIter] Turning left (0.3) for " << (int)(target_angle/0.3) << "s to face the angle: " << target_angle << std::endl;
		} else {
			common::BasicMotion on_spot_aim{0, -0.3, (int)(1000*target_angle/0.3)};
			_plan.push_back(on_spot_aim);
			_new_plan = true;
			std::cout << "[HP->runIter] New target point detected at (" << currentTargetPosition.x << ", " << currentTargetPosition.y << ")" << std::endl; 
			std::cout << "[HP->runIter] Turning right (-0.3) for " << (int)(target_angle/0.3) << "s to face the angle: " << target_angle << std::endl;
		}
		*/
		if (target_angle < Pi){
		    target_angle += Pi;
		}
		//common::BasicMotion on_spot_aim{0, -0.5, (int)(1000*target_angle/0.5)};
		//_plan.push_back(on_spot_aim);
		_new_plan = true;
		std::cout << "[HP->runIter] New target point detected at (" << currentTargetPosition.x << ", " << currentTargetPosition.y << ")" << std::endl; 
		//std::cout << "[HP->runIter] Turning right (-0.3) for " << (int)(target_angle/0.3) << "s to face the angle: " << target_angle << std::endl;
		
		//return;
	}
	
	getLocalTargetPosition();

	// If already close to the target position, or there is no target position, randomly navigate
	std::cout << "Endpoint distance: " << sqrt(pow(local_target_pose.x,2) + pow(local_target_pose.y,2)) << std::endl;
	float redZone = 1;
	if (randomFlag || (sqrt(pow(local_target_pose.x,2) + pow(local_target_pose.y,2)) < redZone)){
		std::cout << "[HP->runIter] TOO CLOSE TO THE ENDPOINT! Within " << sqrt(pow(local_target_pose.x,2) + pow(local_target_pose.y,2)) << "m. Danger zone is: " << redZone << "m." << std::endl;
		std::cout << "[HP->runIter] Jumping over to RandomPlanner now!\n" << std::endl;
		randomFlag = true;
		PrimitivePlanner::runIteration();
		return;
	}
    
    int planned_path = -1;
	int num_on_spots = 2;
	//int num_on_spots = getNumberOnSpots();
	std::cout << "\n" << std::endl;
    pathOptions optionsArray[_primitives.getLength()-num_on_spots];
    for (int plan_index = 0; plan_index < _primitives.getLength()-num_on_spots; plan_index++) {
		bool success = checkPath(_primitives.getPath(plan_index, common::BASE));

        nav_msgs::Path temp_path = _primitives.getPath(plan_index, common::BASE);
        float x = temp_path.poses[temp_path.poses.size()-1].pose.position.x;
        float y = temp_path.poses[temp_path.poses.size()-1].pose.position.y;

        // Expecting next_position struct style with .x and .y fields
        float euclid_dist = sqrt(pow(local_target_pose.x-x,2) + pow(local_target_pose.y-y,2));
        //float euclid_dist = 0;  // while I wait for input

        optionsArray[plan_index] = {plan_index, success, euclid_dist};	
		std::cout << "[HP->runIter] Checking path " << plan_index << std::endl;
		std::cout << "\tto (" << local_target_pose.x << ", " << local_target_pose.y << ")" << std::endl;
		std::cout << "\tsuccess: " << success << ", path end point: (" << x << ", " << y << "), euclid_dist: " << euclid_dist << std::endl;	
	}


	    //for (int i = 0; i < _primitives.getLength()-num_on_spots; i++) {
		//std::cout << "Vector presort"<< optionsArray[i].index << ", success: " << optionsArray[i].valid << ", dist: " << optionsArray[i].euclid_dist << endl;
	//}


    // Sort the potential paths by euclidean distance (increasing order) from the next_position
    std::vector<pathOptions> optionsVector (optionsArray, optionsArray + _primitives.getLength()-num_on_spots);
    std::sort (optionsVector.begin(), optionsVector.end(), boolComparison);

	    for (int i = 0; i < _primitives.getLength()-num_on_spots; i++) {
		std::cout << "Vector postsort"<< optionsVector[i].index << ", success: " << optionsVector[i].valid << ", dist: " << optionsVector[i].euclid_dist << endl;
	}

    for (int i = 0; i < _primitives.getLength()-num_on_spots; i++) {
        if (optionsVector[i].valid){
            planned_path = optionsVector[i].index;
            //std::cout << "Checked path number: " << planned_path << std::endl;
    		_vis.publishPath(_primitives.getPath(planned_path, common::BASE), std::chrono::milliseconds(10));
			std::cout << "[HP->runIter] Chose path: " << optionsVector[i].index << " => success: " << optionsVector[i].valid << ", euclid_dist: " << optionsVector[i].euclid_dist << std::endl;
			
			break;
        }
    }

    // If none of the paths are valid, spin on the spot
    // Call leftOrRightWhileStuck to decide left or right
	// Note: This depends on the last 2 paths being left and right
    if (planned_path < 0){
        planned_path = _primitives.getLength()-num_on_spots;
		bool result = leftOrRightWhileStuck();
		std::cout << "[HP->runIter] No good path. Turning on the spot (0=L, 1=R): " << result << std::endl;
		planned_path += result;
	}

	// add the primitive motion to the plan
	_plan.push_back(_primitives.getMotion(planned_path));

	//add the associated path to the path
	nav_msgs::Path cur_path = _primitives.getPath(planned_path, common::BASE);
	_path.poses.insert(_path.poses.end(), cur_path.poses.begin(), cur_path.poses.end());

	_new_plan = true;
}

void HeuristicPlanner::getLocalTargetPosition(){
	/*  Return the target pose in the local frame of reference 
		r_pi = R*r_pv + r_vi 	=>	 r_pv = R_inv*(r_pi - r_vi)	 =>	 r_pv = R_inv*delta_vector
		r_pi = global point (currentTargetPosition)
		r_vi = odometry measurements (current_pose)
		R = [cos_theta	-sin_theta		=> R_inv = [cos_theta	sin_theta
			 sin_theta	cos_theta]				    -sin_theta	cos_theta]
	*/	
	
	double deltaX = currentTargetPosition.x - current_pose.x;
	double deltaY = currentTargetPosition.y - current_pose.y;

	local_target_pose.x = deltaX*cos(current_pose.theta) + deltaY*sin(current_pose.theta);
	local_target_pose.y = -deltaX*sin(current_pose.theta) + deltaY*cos(current_pose.theta);
	
	std::cout << "[HP->localTarget] Global target point: (" << currentTargetPosition.x << ", " << currentTargetPosition.y << ", " << currentTargetPosition.theta << ")" << std::endl;
	std::cout << "[HP->localTarget] Current global pose: (" << current_pose.x << ", " << current_pose.y << ", " << current_pose.theta << ")" << std::endl;
	std::cout << "[HP->localTarget] Local point: (" << local_target_pose.x << ", " << local_target_pose.y << ", " << local_target_pose.theta << ")" << std::endl;
}

bool HeuristicPlanner::leftOrRightWhileStuck(){
	/*  Choose to turn in place left or right
		From straight ahead, sum up the distances of buckets to the right or left
		Pick the one with the max value (more room to move)
		Returns 0 for left, and 1 for right
	*/

    std::cout << "Radn num:  " << rand_num << std::endl;
	return (rand_num > 0.5);

    return 0;   // Always turn left - make sure it won't get stuck in a corner
    
	int laserSize = (_scan->angle_max -_scan->angle_min)/_scan->angle_increment;
	float left_distances = 0;
	float right_distances = 0;
	for (int i = 1; i <= laserSize; i++){
		if(i<= laserSize/2) {
			if(!isnan(_scan->ranges[i])) { 
				left_distances += _scan->ranges[i];
			}
		} 
		else {
			if(!isnan(_scan->ranges[i])) { 
				right_distances += _scan->ranges[i];
			}
		}	
	}

	std::cout << "[HP->L/R Stuck] Left dist: " << left_distances << ", right dist: " << right_distances << std::endl;
	return (left_distances < right_distances);	// 0 if left >= right to go left, 1 if left < right to go right
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

	//int backupPathIndex;
    int max_path = -1;
    float max_outcome = 0;
	for (int i = 0; i < _primitives.getLength()-1; i++) {
		// Hacked way to see if it is on_spot		
		//if (_primitives.getPath(i, common::BASE).linear_velocity == 0){
		//nav_msgs::Path tempPath = _primitives.getPath(i, common::BASE);
		//if (tempPath.poses[0].pose.position == tempPath.poses[tempPath.poses.size()-1].pose.position){
		//	backupPathIndex = i;
		//	continue;
		//}
		float outcome = checkPath(_primitives.getPath(i, common::BASE));    
        //std::cout << "Checked path number: " << i << std::endl;   
		_vis.publishPath(_primitives.getPath(i, common::BASE), std::chrono::milliseconds(10));
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
	double dist = sqrt(pow(curr_position[0]-x,2) + pow(curr_position[1]-y,2)) + 0.05;
	return atan2(robotRadius, dist);
}

bool PrimitivePlanner::checkObstacle(float x_pos, float y_pos, float scan_angle){
	/* Given (x_pos,y_pos) and a scan angle, determine whether there is an obstacle there. return 
			- true if there is an obstacle
			- false if there is no obstacle
	*/	
	
	// take the (x_pos,y_pos) and calculate the angle and tangent.
	float angle = atan2(y_pos,x_pos);
	float tangent = sqrt((y_pos)*(y_pos) + (x_pos)*(x_pos));
	
	if (_scan->angle_max < angle || _scan->angle_min > angle){		
		// outside of the viewing angle is not an obstacle		
		//std::cout<<"outside of viewing angle"<<std::endl;
		return false;
	}
	else if (_scan->range_min > tangent || _scan->range_max < tangent){	
		// anything outside of the range is not an obstacle		
		//std::cout<<"outside of range"<<std::endl;
		return false;
	}
	else {
		// Check (x_pos,y_pos) position for obstacle.
		int index = (angle - (_scan->angle_min))/(_scan->angle_increment);
		int laserSize = (_scan->angle_max -_scan->angle_min)/_scan->angle_increment;
		int scan_width = (scan_angle)/_scan->angle_increment;
		
		//std::cout << "we have " << _scan->ranges.size() << std::endl;
		//std::cout << "scan width: " << scan_width << ". laserSize: " << laserSize << std::endl;
		
	    
	    for(int i = index-scan_width; i <= index+scan_width; i++){
			if(i >= 0 && i < laserSize){
				if(tangent > (_scan->ranges[i] - 0.3)){
					return true;	
				}
			}
        }
		/*for(int i = index-scan_width; i <= index+scan_width; i++){
		    std::cout << _scan->ranges[i] << " ";
			if(i >= 0 && i < laserSize){
				//if(tangent > (_scan->ranges[i]-0.3) || std::isnan(_scan->ranges[i])){
				if(std::isnan(_scan->ranges[i]) || tangent > (_scan->ranges[i])-0.3){					
					_vis.publishErrorPoint(x_pos, y_pos, std::chrono::milliseconds(1));
					float test_angle = i * _scan->angle_increment + _scan->angle_min;
					_vis.publishLaserPoint(_scan->ranges[i], test_angle, std::chrono::milliseconds(1));
					
					std::cout << _scan->ranges[i+1] << " "<< _scan->ranges[i+2] << " "<< _scan->ranges[i+3] << " "<< _scan->ranges[i+4] << " "<< _scan->ranges[i+5] << " "<< _scan->ranges[i+6] << " "<< _scan->ranges[i+7] << " ";
					std::cout << endl;
					
					std::cout << "Found issue at positin [" << x_pos << "," << y_pos << "], index [" << i << "] with range [" << _scan->ranges[i] << "], angle [" << test_angle << "]" << std::endl;
					//std::cout << "Maximum laserSize: " << laserSize << std::endl;
					return true; 	
				}
			}
		} */
	}
	return false;
}

int PrimitivePlanner::ifObstacle(){
	/* This function is used as an interrupt.
		At any point, if there is an obstacle, it'll start re planning a path.
		This includes laser scans and bumper.
		It returns: 
			- 0 for left bump
			- 1 for center bump
			- 2 for right bump
			- 3 for scanned obstacle within 0.5
			- 4 for nothing
	*/	

	int laserSize = (_scan->angle_max -_scan->angle_min)/_scan->angle_increment;
	int turn_right = 0;
	int turn_left = 0;
	for (int i = 1; i <= laserSize; i++){
		if(_scan->ranges[i] < 0.5 ) {
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

	int pose_points = path.poses.size();
	
    //std::cout << "Checking " << pose_points << " points along the path" << endl;
    
	for (int i = 0; i < pose_points; i++){
		float x = path.poses[i].pose.position.x;
		float y = path.poses[i].pose.position.y;
		float scan_angle = scanWidthAngle(path, x, y);
		//std::cout << "(" <<x << "," <<y <<")" << endl;
		//std::cout << "i:" << i << ". scanAngle: " << scan_angle << endl;
		//float scan_angle = 2;
		if (checkObstacle(x, y, scan_angle)){
		    //std::cout << "(" <<x << "," <<y <<")" << endl;
		    //std::cout << "i:" << i << ". scanAngle: " << scan_angle << endl;
			//std::cout << "invalid path" << std::endl;
			return 0;
		}
	}

	//std::cout << "valid path" << std::endl;
	return 1;
}


float WeightedPlanner::scanWidthAngle(float curr_x, float curr_y, float x, float y){
	  /*This is a helper function for checkPath, to be passed to checkObstacle
		The idea is to find the angle between the robot's current position, and the point being checked
		This is to help tally up the min/max angle bounds for diving paths into laser bin ranges
		*/
    std::cout << "SHOULD NEVER COME HERE" << endl;
	double curr_position[2] = {curr_x, curr_y};
	return atan2(curr_position[1]-y, curr_position[0]-x);
}

float WeightedPlanner::getDistance(float max_angle, float min_angle){
	 /*
        Returns:
            -1 if invalid path (out of FOV or obstacles hit)
            Otherwise returns sum of all buckets in range
		*/
	
	std::cout << "SHOULD NEVER COME HERE" << endl;
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

    std::cout << "SHOULD NEVER COME HERE" << endl;
    
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

