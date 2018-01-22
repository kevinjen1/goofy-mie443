#pragma once

#include<nav_msgs/Path.h>
#include<vector>
#include<memory>
#include<iostream>

#include"common/common.hpp"

namespace goofy{
namespace planner{

typedef std::vector<common::BasicMotion> MotionList;
typedef std::vector<nav_msgs::Path> PathList;

class PrimitiveRepresentation{
public:
	typedef std::shared_ptr<PrimitiveRepresentation> SharedPtr;

	PrimitiveRepresentation(common::RobotModel robot, MotionList inputs):
	_model(robot),
	_inputs(inputs){}

	nav_msgs::Path getPath(int index, const std::string& desired_frame);
	common::BasicMotion getMotion(int index);

private:
	common::RobotModel _model;
	MotionList _inputs;
	PathList _path_list;
};

}
}
