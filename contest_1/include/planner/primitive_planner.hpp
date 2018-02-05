#pragma once

#include<ros/ros.h>
#include<geometry_msgs/Twist.h>

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
		_new_plan(false){
		_path.header.frame_id = "camera_depth_frame";
	}
	virtual ~PrimitivePlanner() = default;

	MotionList getPlan(){
		return _plan;
	}

	nav_msgs::Path getPath(){
		return _path;
	}

	virtual void runIteration() = 0;

	void updateLaserScan(sensor_msgs::LaserScan::ConstPtr msg){
		_scan = msg;
	}

	bool getVelocity(geometry_msgs::Twist& vel);
	bool bumperLeft = 0, bumperCenter = 0, bumperRight = 0;
	double laserRange = 10;
	int laserSize = 0, laserOffset = 0, desiredAngle = 5;
	float robotRadius = 0.2;	// in m

protected:
	bool checkObstacle(float x_pos, float y_pos, float scan_angle);
	bool ifObstacle();
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
	std::chrono::steady_clock::time_point _end_motion_time;
};

class RandomPlanner: public PrimitivePlanner{
public:
	RandomPlanner(PrimitiveRepresentation primitives):
		PrimitivePlanner(primitives){}

	virtual void runIteration() override;
};

class HeuristicPlanner: public PrimitivePlanner{
public:
	HeuristicPlanner(PrimitiveRepresentation primitives):
		PrimitivePlanner(primitives){}

	virtual void runIteration() override;

    struct pathOptions {
        int index;
        bool valid;
        float euclid_dist;
    };

    static bool boolComparison(pathOptions i, pathOptions j);
};

class WeightedPlanner: public PrimitivePlanner{
public:
	WeightedPlanner(PrimitiveRepresentation primitives):
		PrimitivePlanner(primitives){}

	virtual void runIteration() override;

    float checkPath(nav_msgs::Path path);
    float scanWidthAngle(float curr_x, float curr_y, float x, float y);
    float getDistance(float max_angle, float min_angle);
};

}
}
