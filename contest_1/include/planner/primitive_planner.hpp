#pragma once

#include<ros/ros.h>
#include<geometry_msgs/Twist.h>

#include"primitives/generator.hpp"
#include"common/visualizer.hpp"
#include"common/common.hpp"

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

protected:
	bool checkPath(nav_msgs::Path path){
		return true;
	}

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
