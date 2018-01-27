#include "mapper/mapper.hpp"

namespace goofy{
namespace common{

void processRawMap(const nav_msgs::OccupancyGrid grid){
  int width = grid.info.width;
  int height = grid.info.height;

  int minWidth = 0;
  int maxWidth = 0;

  int minHeight = 0;
  int maxHeight = 0;

  int boundingWidth = 0;
  int boundingHeight = 0;

  bool found = false;
  for (int i=0; i<height; i++) {
    for (int j=0; j<width; j++) {
      int index = i*width+j;
      if (grid.data[index] >= 0){
        minHeight = i;
        found = true;
        break;
      }
    }
    if (found) {
      break;
    }
  }

  found = false;
  for (int i=height-1; i>=0; i--) {
    for (int j=0; j<width; j++) {
      int index = i*width+j;
      if (grid.data[index] >= 0){
        maxHeight = i;
        found = true;
        break;
      }
    }
    if (found) {
      break;
    }
  }

  found = false;
  for (int i=0; i<width; i++) {
    for (int j=0; j<height; j++) {
      int index = j*width+i;
      if (grid.data[index] >= 0){
        minWidth = i;
        found = true;
        break;
      }
    }
    if (found) {
      break;
    }
  }

  found = false;
  for (int i=width-1; i>=0; i--) {
    for (int j=0; j<height; j++) {
      int index = j*width+i;
      if (grid.data[index] >= 0){
        maxWidth = i;
        found = true;
        break;
      }
    }
    if (found) {
      break;
    }
  }
  boundingHeight = maxHeight - minHeight + 1;
  boundingWidth = maxWidth - minWidth + 1;

  int robotPosX = grid.info.origin.position.x;



  ROS_INFO("I have a grid");
  ROS_INFO("%i %i %f",grid.info.width, grid.info.height, grid.info.resolution);
  ROS_INFO("%f %f %f",grid.info.origin.position.x, grid.info.origin.position.y, grid.info.origin.position.z);
  ROS_INFO("calculated width height");
  ROS_INFO("%i %i", boundingWidth, boundingHeight);
  //int8[] data = grid.data;
  //ROS_INFO();
}

}
}
