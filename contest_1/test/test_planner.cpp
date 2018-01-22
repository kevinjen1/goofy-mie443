#include<gtest/gtest.h>
#include<ros/ros.h>

#include"common/visualizer.hpp"
#include"common/common.hpp"

#include"planner/primitive_planner.hpp"

using namespace goofy;

TEST(PrimitiveTests, PathGeneration){
	common::Visualizer vis;
	common::RobotModel model;

}

int main(int argc, char **argv){
	ros::init(argc, argv, "test_visualizer", ros::init_options::AnonymousName);
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
