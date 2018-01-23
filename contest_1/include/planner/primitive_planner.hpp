#pragma once

#include"primitives/generator.hpp"
#include"common/common.hpp"

namespace goofy{
namespace planner{

const double DENSITY = 0.3;

class PrimitivePlanner{
public:
	PrimitivePlanner(PrimitiveRepresentation primitives):
		_primitives(primitives){}

	MotionList getPlan();
	void runIteration();

private:
	bool checkObstacle(const sensor_msgs::LaserScan::ConstPtr& msg, float x_pos, float y_pos);

	MotionList _plan;
	PrimitiveRepresentation _primitives;

};

}
}
