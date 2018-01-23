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
		_path.header.frame_id = common::BASE;
	}
	virtual ~PrimitivePlanner() = default;

	MotionList getPlan(){
		return _plan;
	}

	nav_msgs::Path getPath(){
		return _path;
	}

	virtual void runIteration() = 0;

	bool getVelocity(geometry_msgs::Twist& vel);
	bool bumperLeft = 0, bumperCenter = 0, bumperRight = 0;
	double laserRange = 10;
	int laserSize = 0, laserOffset = 0, desiredAngle = 5;

	static bool checkObstacle(const sensor_msgs::LaserScan::ConstPtr& msg, float x_pos, float y_pos);

protected:

	bool checkPath(nav_msgs::Path path);
	bool checkObstacle(const sensor_msgs::LaserScan::ConstPtr& msg, float x_pos, float y_pos);

	common::Visualizer _vis;
	MotionList _plan;
	nav_msgs::Path _path;
	PrimitiveRepresentation _primitives;
	bool _new_plan;

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

}
}
