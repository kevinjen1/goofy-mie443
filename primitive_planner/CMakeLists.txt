cmake_minimum_required(VERSION 2.8)
project(primitive_planner)

find_package(catkin REQUIRED COMPONENTS 
	cv_bridge 
	genmsg 
	image_transport 
	sensor_msgs 
	kobuki_msgs)

generate_messages(DEPENDENCIES sensor_msgs kobuki_msgs)

catkin_package()

include_directories(include ${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

# add the publisher example
add_executable(motion_generation 
	src/generator/generator_module.cpp
	src/generator/generator.cpp)

target_link_libraries(motion_generation ${catkin_LIBRARIES})
