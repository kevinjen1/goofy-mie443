#include<gtest/gtest.h>
#include<ros/ros.h>
#include<chrono>
#include<thread>


#include <func_header.h>

TEST(PrimitiveTests, PathGeneration){
	ROS_INFO("hello there");
}

int main(int argc, char **argv){
	ros::init(argc, argv, "test_pic_sim", ros::init_options::AnonymousName);
	testing::InitGoogleTest(&argc, argv);

	return RUN_ALL_TESTS();
}
