cmake_minimum_required(VERSION 2.8)
project(mie443_contest3)

set(CMAKE_CXX_STANDARD 11)

find_package(OpenCV 3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
		cv_bridge 
		image_transport 
		sensor_msgs
		kobuki_msgs
		sound_play
		)

catkin_package()

include_directories(include ${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_definitions(-DROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

# add the publisher example
add_executable(publisher src/my_publisher_video.cpp)
target_link_libraries(publisher ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

# add the subscriber example
add_executable(contest3 src/contest3.cpp
			src/imageTransporter.cpp
			src/fsm.cpp
			src/PlayVideo.cpp)
target_link_libraries(contest3 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

# add the test
catkin_add_gtest(test_threading 
	test/test_threading.cpp
	src/PlayVideo.cpp
	src/fsm.cpp
)

target_link_libraries(test_threading 
	${catkin_LIBRARIES} 
	${OpenCV_LIBRARIES}
)
