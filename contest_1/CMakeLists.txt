cmake_minimum_required(VERSION 2.8)
project(mie443_contest1)

set(CMAKE_CXX_STANDARD 11)

#set(OpenCV_DIR "/usr/share/OpenCV")

find_package(OpenCV 3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
	cv_bridge
	genmsg
	image_transport
	sensor_msgs
	kobuki_msgs
	nav_msgs
	message_generation
	tf)

add_message_files (
	FILES
	customMsg.msg
)

generate_messages(DEPENDENCIES sensor_msgs kobuki_msgs)

catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(include ${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_library(goofy_common src/common/common.cpp src/common/visualizer.cpp)
target_link_libraries(goofy_common ${catkin_LIBRARIES})

add_library(primitive_planner src/planner/primitive_planner.cpp src/primitives/generator.cpp)
target_link_libraries(primitive_planner goofy_common ${catkin_LIBRARIES})

add_library(mapper src/mapper/mapper.cpp)
target_link_libraries(mapper ${catkin_LIBRARIES} ${OpenCV_LIB})

# catkin_add_gtest(test_mapper test/test_mapper.cpp)
# target_link_libraries(test_mapper mapper)

# add the publisher example
add_executable(contest1 src/contest1.cpp)
target_link_libraries(contest1 ${catkin_LIBRARIES} ${OpenCV_LIB} goofy_common primitive_planner)

catkin_add_gtest(test_planner test/test_planner.cpp)
target_link_libraries(test_planner primitive_planner goofy_common)

catkin_add_gtest(test_common test/test_common.cpp)
target_link_libraries(test_common goofy_common)

add_executable(goofMapper src/mapper/mapper.cpp)
target_link_libraries(goofMapper ${catkin_LIBRARIES} goofy_common)
add_executable(goofNavigator src/mapper/navigator.cpp)
target_link_libraries(goofNavigator ${catkin_LIBRARIES} goofy_common)

add_executable(depth_processor src/depth/depth.cpp)
target_link_libraries(depth_processor ${catkin_LIBRARIES} ${OpenCV_LIB} goofy_common)
