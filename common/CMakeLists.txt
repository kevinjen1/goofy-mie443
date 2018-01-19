cmake_minimum_required(VERSION 2.8)
project(common_resources)

set(goofy_INCLUDE_DIRS ${CMAKE_CURRENT_BINARY_DIR}/include)

find_package(OpenCV 3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS 
	cv_bridge 
	genmsg 
	image_transport 
	sensor_msgs 
	kobuki_msgs)

catkin_package()

include_directories(include ${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

# add the publisher example
add_library(goofy_common src/common.cpp)
target_link_libraries(goofy_common ${catkin_LIBRARIES} ${OpenCV_LIB})
