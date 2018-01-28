#include<gtest/gtest.h>
#include<ros/ros.h>
#include<chrono>
#include<thread>


#include"mapper/mapper.hpp"

using namespace goofy;

TEST(PrimitiveTests, PathGeneration){
	ROS_INFO("hello there");

}

int main(int argc, char **argv){
	ros::init(argc, argv, "test_mapper", ros::init_options::AnonymousName);
	testing::InitGoogleTest(&argc, argv);



	return RUN_ALL_TESTS();
}
