#include<gtest/gtest.h>
#include<ros/ros.h>
#include<chrono>
#include<thread>

#include<sensor_msgs/LaserScan.h>

#include"common/visualizer.hpp"
#include"common/common.hpp"

using namespace goofy;

TEST(Filters, LaserScanNANFilter){
	sensor_msgs::LaserScan scan;
	scan.intensities = {nan, nan, 1.05, 1.8, 2.0, 2.0, nan, nan, nan, 3.0, nan, nan};

	sensor_msgs::LaserScan corr;
	corr.intensities = {nan, nan, 1.05, 1.8, 2.0, 2.0, 2.5, 2.5, 2.5, 3.0, nan, nan};

	common::filterLaserScan(scan, 4);
	ASSERT_EQ(scan.intensities, corr.intensities);
}

int main(int argc, char **argv){
	ros::init(argc, argv, "test_visualizer", ros::init_options::AnonymousName);
	testing::InitGoogleTest(&argc, argv);

	return RUN_ALL_TESTS();
}

