#include<gtest/gtest.h>
#include<ros/ros.h>
#include<chrono>
#include<thread>

#include"common/visualizer.hpp"
#include"common/common.hpp"

#include"planner/primitive_planner.hpp"

using namespace goofy;

TEST(PrimitiveTests, PathGeneration){
	common::Visualizer vis;
	common::RobotModel model(0.5,0.5,0.5);

	nav_msgs::Path path = model.simulatePath(0.3, 0.3, 3, 0.05);
	vis.publishPath(path, std::chrono::seconds(1));
}

int main(int argc, char **argv){
	ros::init(argc, argv, "test_visualizer", ros::init_options::AnonymousName);
	testing::InitGoogleTest(&argc, argv);

	return RUN_ALL_TESTS();
}
