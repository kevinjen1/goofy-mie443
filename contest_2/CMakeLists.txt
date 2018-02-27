cmake_minimum_required(VERSION 2.8.3)
project(mie443_contest2)

find_package(OpenCV 3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  	actionlib
  	move_base_msgs
  	roscpp
	cv_bridge
	image_transport
	gazebo_ros
)

find_package(orocos_kdl REQUIRED)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(contest2 src/contest2.cpp
			src/img_proc.cpp
			src/accessory/init.cpp
			src/accessory/imageTransporter.cpp)

target_link_libraries(contest2 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} orocos-kdl)

add_executable(webcam_publisher src/accessory/my_publisher_video.cpp)
target_link_libraries(webcam_publisher ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
