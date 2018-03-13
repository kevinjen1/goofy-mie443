#include<gtest/gtest.h>
#include<ros/ros.h>
#include<chrono>
#include<thread>
#include<vector>


#include <func_header.h>

TEST(AreaTests, Square){
	std::vector<cv::Point2f> points;
	points.push_back(cv::Point2f(1,1));
	points.push_back(cv::Point2f(1,2));
	points.push_back(cv::Point2f(2,2));
	points.push_back(cv::Point2f(2,1));

	ASSERT_FLOAT_EQ(1, getArea(points));
}

TEST(AreaTests, NonConvex){
	std::vector<cv::Point2f> points;
	points.push_back(cv::Point2f(1,1));
	points.push_back(cv::Point2f(2,2));
	points.push_back(cv::Point2f(3,0));
	points.push_back(cv::Point2f(2,1));

	ASSERT_FLOAT_EQ(1, getArea(points));
}

int main(int argc, char **argv){
	ros::init(argc, argv, "test_pic_sim", ros::init_options::AnonymousName);
	testing::InitGoogleTest(&argc, argv);

	return RUN_ALL_TESTS();
}
