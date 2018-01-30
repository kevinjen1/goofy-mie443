//#pragma once
#include "mapper/mapper.hpp"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"

namespace goofy {
namespace mapper {

class MapProcessing {
public:

	MapProcessing() {
		pub = n.advertise<nav_msgs::OccupancyGrid>("goofMap", 1);
		sub = n.subscribe("map", 1, &MapProcessing::callback, this);
	}

	void callback(const nav_msgs::OccupancyGrid grid) {
		int width = grid.info.width;
		int height = grid.info.height;

		int minWidth;
		int maxWidth;

		int minHeight;
		int maxHeight;

		int localWidth = 0;
		int localHeight = 0;

		// get a bounding box around the known sections of the map
		minHeight = getMinHeight(width, height, grid.data);
		maxHeight = getMaxHeight(width, height, grid.data);
		minWidth = getMinWidth(width, height, grid.data);
		maxWidth = getMaxWidth(width, height, grid.data);

		nav_msgs::OccupancyGrid localMap;

		// if any of them is -1 (then all should be -1) and it means nothing is known
		if (minHeight < 0 || maxHeight < 0 || minWidth < 0 || maxWidth < 0) {
			return;
		}

		// get the size of the bounding box
		localHeight = maxHeight - minHeight + 1;
		localWidth = maxWidth - minWidth + 1;

		vector<int8_t> matrix;
		matrix.resize(localHeight*localWidth);
		for (int i = 0; i < localHeight; i++) {
			for (int j = 0; j < localWidth; j++) {
				int localIndex = i*localWidth + j;
				int index = (i + minHeight) * width + (j + minWidth);
				matrix[localIndex] = grid.data[index];
			}
		}

		geometry_msgs::Pose pose = grid.info.origin;
		// THIS IS NOT CORRECT. NEED TO CONSIDER ORIENTATION AS WELL
		pose.position.x += minWidth * grid.info.resolution;
		pose.position.y += minHeight * grid.info.resolution;

		localMap.info.height = localHeight;
		localMap.info.width = localWidth;
		localMap.info.resolution = grid.info.resolution;
		localMap.info.map_load_time = grid.info.map_load_time;
		localMap.info.origin = pose;
		localMap.data = matrix;

	//	ROS_INFO("I have a grid");
	//	ROS_INFO("%i %i %f", grid.info.width, grid.info.height,
	//			grid.info.resolution);
	//	ROS_INFO("%f %f %f", grid.info.origin.position.x,
	//			grid.info.origin.position.y, grid.info.origin.position.z);
	//	ROS_INFO("calculated width height");
	//	ROS_INFO("%i %i", boundingWidth, boundingHeight);

		pub.publish(localMap);
	}

private:
	ros::NodeHandle n;
	ros::Publisher pub;
	ros::Subscriber sub;
};

int getMinHeight(int width, int height, vector<int8_t> data) {
	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			int index = i * width + j;
			if (data[index] >= 0) {
				return i;
			}
		}
	}
	return -1;
}

int getMaxHeight(int width, int height, vector<int8_t> data) {
	for (int i = height - 1; i >= 0; i--) {
		for (int j = 0; j < width; j++) {
			int index = i * width + j;
			if (data[index] >= 0) {
				return i;
			}
		}
	}
	return -1;
}

int getMinWidth(int width, int height, vector<int8_t> data) {
	for (int i = 0; i < width; i++) {
		for (int j = 0; j < height; j++) {
			int index = j * width + i;
			if (data[index] >= 0) {
				return i;
			}
		}
	}
	return -1;
}

int getMaxWidth(int width, int height, vector<int8_t> data) {
	for (int i = width - 1; i >= 0; i--) {
		for (int j = 0; j < height; j++) {
			int index = j * width + i;
			if (data[index] >= 0) {
				return i;
			}
		}
	}
	return -1;
}

}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "goofMapper");
	goofy::mapper::MapProcessing subAndP;

	ros::spin();

	return 0;
}

