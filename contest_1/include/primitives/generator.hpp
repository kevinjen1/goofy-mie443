#pragma once

#include<nav_msgs/Path.h>
#include<vector>
#include<memory>
#include<iostream>

#include"common/common.hpp"

namespace goofy{
namespace planner{

const double DENSITY = 0.01;

typedef std::vector<common::BasicMotion> MotionList;
typedef std::vector<nav_msgs::Path> PathList;

class PrimitiveRepresentation{
public:
	typedef std::shared_ptr<PrimitiveRepresentation> SharedPtr;

	PrimitiveRepresentation(common::RobotModel robot, MotionList inputs):
	_model(robot),
	_inputs(inputs),
	_size(inputs.size()){
		//generate all the paths
		for (MotionList::iterator i = inputs.begin(); i != inputs.end(); i++){
			_path_list.push_back(robot.simulatePath(*i, DENSITY));
		}
	}

	int getLength(){
		return _size;
	}

	nav_msgs::Path getPath(int index, const std::string& desired_frame){
		if (index < _size){
			return _path_list[index];
		} else {
			return _path_list[0];
		}
	}
	common::BasicMotion getMotion(int index){
		if (index < _size){
			return _inputs[index];
		} else {
			return _inputs[0];
		}
	}

private:
	int _size;
	common::RobotModel _model;
	MotionList _inputs;
	PathList _path_list;
};

}
}
