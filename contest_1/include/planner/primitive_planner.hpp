#pragma once

#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>

#include <sensor_msgs/LaserScan.h>
#include"primitives/generator.hpp"
#include"common/visualizer.hpp"
#include"common/common.hpp"

#define Pi 3.14159265

namespace goofy{
namespace planner{

class PrimitivePlanner{
public:
	PrimitivePlanner(PrimitiveRepresentation primitives):
		_primitives(primitives),
		_new_plan(false),
		_recovery(false){
		_path.header.frame_id = "camera_depth_frame";
		nextPosition.x = 0;
		nextPosition.y = 0;
	}
	virtual ~PrimitivePlanner() = default;

	MotionList getPlan(){
		return _plan;
	}

	nav_msgs::Path getPath(){
		return _path;
	}

	void runIteration();

	void updateLaserScan(sensor_msgs::LaserScan::ConstPtr msg){
		_scan = msg;
	}

	bool getVelocity(geometry_msgs::Twist& vel);
	bool bumperLeft = 0, bumperCenter = 0, bumperRight = 0;
	double laserRange = 10;
	int laserSize = 0, laserOffset = 0, desiredAngle = 5;
	float robotRadius = 0.2;	// in m
	geometry_msgs::Pose2D nextPosition;
	geometry_msgs::Pose2D current_pose;
	geometry_msgs::Pose2D currentTargetPosition;
	geometry_msgs::Pose2D local_target_pose;

protected:
	bool checkObstacle(float x_pos, float y_pos, float scan_angle);
	int ifObstacle();
	bool checkPath(nav_msgs::Path path);
	double shortenPathTimeTo(nav_msgs::Path path);
	float scanWidthAngle(nav_msgs::Path path, float x, float y);

	common::Visualizer _vis;
	MotionList _plan;
	nav_msgs::Path _path;
	PrimitiveRepresentation _primitives;
	bool _new_plan;
	sensor_msgs::LaserScan::ConstPtr _scan;

private:
	geometry_msgs::Twist _vel;
	common::BasicMotion _curr_motion;
	MotionList::iterator _motion_index;
	bool _recovery;
	std::chrono::steady_clock::time_point _end_motion_time;
};

class HeuristicPlanner: public PrimitivePlanner{
public:
	HeuristicPlanner(PrimitiveRepresentation primitives):
		PrimitivePlanner(primitives){}

	void runIteration();

    struct pathOptions {
        int index;
        bool valid;
        float euclid_dist;
    };

    static bool boolComparison(pathOptions i, pathOptions j);
	int getNumberOnSpots();
	bool leftOrRightWhileStuck();
	void getLocalTargetPosition();
	
	bool randomFlag = false;
};

class WeightedPlanner: public PrimitivePlanner{
public:
	WeightedPlanner(PrimitiveRepresentation primitives):
		PrimitivePlanner(primitives){}

	void runIteration();

    float checkPath(nav_msgs::Path path);
    float scanWidthAngle(float curr_x, float curr_y, float x, float y);
    float getDistance(float max_angle, float min_angle);
};

}
}
