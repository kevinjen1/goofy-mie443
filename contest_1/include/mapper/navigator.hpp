//#ifndef GOOFY_MIE443_CONTEST_1_INCLUDE_MAPPER_NAVIGATOR_HPP_
//#define GOOFY_MIE443_CONTEST_1_INCLUDE_MAPPER_NAVIGATOR_HPP_

#include "geometry_msgs/Pose2D.h"
#include "mapper/mapper.hpp"
#define PI	3.14159265358979323846
namespace goofy{
namespace mapper{

struct Slope {
	double rise;
	double run;
};

geometry_msgs::Pose2D getCoordinateRayCasting(nav_msgs::OccupancyGrid grid, Slope slope, int robotRow, int robotCol, geometry_msgs::Pose2D robotPos);
vector<vector<int>> getMatrixFromGrid(nav_msgs::OccupancyGrid grid);
Slope getClosestAxisToHeading(double theta);
double getAngle(int* angleChange);
bool isZero(double value);
int convertToDegree(double rad);

}}

//#endif /* GOOFY_MIE443_CONTEST_1_INCLUDE_MAPPER_NAVIGATOR_HPP_ */
