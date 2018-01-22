#pragma once

#include"primitives/generator.hpp"
#include"common/common.hpp"

namespace goofy{
namespace planner{

class PrimitivePlanner{
public:
	PrimitivePlanner(PrimitiveRepresentation primitives):
		_primitives(primitives){}

	MotionList getPlan();
	void runIteration();

private:
	bool checkObstacle();

	MotionList _plan;
	PrimitiveRepresentation _primitives;

};

}
}
