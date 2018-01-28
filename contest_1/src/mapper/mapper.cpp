#pragma once
#include "mapper/mapper.hpp"

namespace goofy {
namespace mapper {

/**
 * this should subscribe to the gmap to get OccupancyGrid
 * and publish to a topic, to pass it to navigator
 */

mapper::LocalMap GMap::processRawMap(const nav_msgs::OccupancyGrid grid) {
	int width = grid.info.width;
	int height = grid.info.height;

	int minWidth;
	int maxWidth;

	int minHeight;
	int maxHeight;

	int boundingWidth = 0;
	int boundingHeight = 0;

	// get a bounding box around the known sections of the map
	minHeight = GMap::getMinHeight(width, height, grid.data);
	maxHeight = GMap::getMaxHeight(width, height, grid.data);
	minWidth = GMap::getMinWidth(width, height, grid.data);
	maxWidth = GMap::getMaxWidth(width, height, grid.data);

//	mapper::LocalMap localMap = new mapper::LocalMap;
	mapper::LocalMap localMap = {};

	// if any of them is -1 (then all should be -1) and it means nothing is known
	if (minHeight < 0 || maxHeight < 0 || minWidth < 0 || maxWidth < 0) {
		return localMap;
	}

	// get the size of the bounding box
	boundingHeight = maxHeight - minHeight + 1;
	boundingWidth = maxWidth - minWidth + 1;

	// copy the bounded region to a new array
//	int matrix[boundingHeight][boundingWidth];
	vector<vector<int> > matrix;
	matrix.resize(boundingHeight);
	for (int i = 0; i < boundingHeight; i++) {
		matrix[i].resize(boundingWidth);
		for (int j = 0; j < boundingWidth; j++) {
			int index = (i + minHeight) * width + (j + minWidth);
			matrix[i][j] = grid.data[index];
		}
	}

	geometry_msgs::Pose pose = grid.info.origin;
	// THIS IS NOT CORRECT. nEED TO CONSIDER ORIENTATION AS WELL
	pose.position.x += minWidth * grid.info.resolution;
	pose.position.y += minHeight * grid.info.resolution;

	localMap.boundingHeight = boundingHeight;
	localMap.boundingHeight = boundingHeight;
	localMap.matrix = matrix;
	localMap.resolution = grid.info.resolution;
	localMap.time = grid.info.map_load_time;
	localMap.origin = pose;

	ROS_INFO("I have a grid");
	ROS_INFO("%i %i %f", grid.info.width, grid.info.height,
			grid.info.resolution);
	ROS_INFO("%f %f %f", grid.info.origin.position.x,
			grid.info.origin.position.y, grid.info.origin.position.z);
	ROS_INFO("calculated width height");
	ROS_INFO("%i %i", boundingWidth, boundingHeight);
	//int8[] data = grid.data;
	//ROS_INFO();
	return localMap;
}

int GMap::getMinHeight(int width, int height, vector<int8_t> data) {
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

int GMap::getMaxHeight(int width, int height, vector<int8_t> data) {
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

int GMap::getMinWidth(int width, int height, vector<int8_t> data) {
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

int GMap::getMaxWidth(int width, int height, vector<int8_t> data) {
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
