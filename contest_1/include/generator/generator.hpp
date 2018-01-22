#pragma once

#include<nav_msgs/Path.h>
#include<vector>

#include"common.hpp"

namespace goofy{
namespace planner{

struct BasicMotion{
	double linear_velocity;
	double angular_velocity;
	double time;
};

typedef std::vector<BasicMotion> MotionList;


class PrimitiveRepresentation{
public:
	PrimitiveRepresentation(common::RobotModel robot, MotionList inputs):
	_model(robot),
	_inputs(inputs){}

private:
	common::RobotModel _model;
	MotionList _inputs;
};

}
}
