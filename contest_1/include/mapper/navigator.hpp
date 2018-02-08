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
struct gridIndex {
	int row;
	int col;

	gridIndex(int _row, int _col) :
			row(_row), col(_col) {
	}

	gridIndex() : row(0), col(0) {
	}
};
struct gridDFSElement {
	int data;
	gridIndex index;
	bool isDiscovered;
	vector<gridIndex> adjList;
};

geometry_msgs::Pose2D getCoordinateRayCasting(nav_msgs::OccupancyGrid grid, Slope slope, int robotRow, int robotCol, geometry_msgs::Pose2D robotPos);
vector<vector<int>> getMatrixFromGrid(nav_msgs::OccupancyGrid grid);
Slope getClosestAxisToHeading(double theta);
double getAngle(int* angleChange);
bool isZero(double value);
int convertToDegree(double rad);
geometry_msgs::Pose2D convertToLocal(geometry_msgs::Pose2D coord, geometry_msgs::Pose2D robPos);

// Depth search functions
geometry_msgs::Pose2D getCoordinateDFS(nav_msgs::OccupancyGrid grid, Slope slope, int robotRow, int robotCol, geometry_msgs::Pose2D robotPos);
geometry_msgs::Pose2D getCoordinateBFS(nav_msgs::OccupancyGrid grid, Slope slope, int robotRow, int robotCol, geometry_msgs::Pose2D robotPos);

vector<vector<gridDFSElement>> getMatrixFromGridDFS(nav_msgs::OccupancyGrid grid, Slope slope);
vector<gridIndex> fillAdjList(vector<vector<gridDFSElement>> matrix, int i, int j, Slope slope, int height, int width);
void pushBackNegX(vector<gridIndex> &v, int i, int j);
void pushBackPosX(vector<gridIndex> &v, int i, int j, int width);
void pushBackNegY(vector<gridIndex> &v, int i, int j);
void pushBackPosY(vector<gridIndex> &v, int i, int j, int height);

}}

//#endif /* GOOFY_MIE443_CONTEST_1_INCLUDE_MAPPER_NAVIGATOR_HPP_ */
