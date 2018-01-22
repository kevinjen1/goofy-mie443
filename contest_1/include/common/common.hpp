#pragma once

#include<nav_msgs/Path.h>

namespace goofy{
	namespace common{

class RobotModel{
public:
	RobotModel(double left_rad, double right_rad, double wheelbase):
		_left_rad(left_rad),
		_right_rad(right_rad),
		_wheelbase(wheelbase){}

	nav_msgs::Path simulatePath(double lin_vel, double ang_vel, double time, double density);

private:
	double _left_rad;
	double _right_rad;
	double _wheelbase;
};

}
}

