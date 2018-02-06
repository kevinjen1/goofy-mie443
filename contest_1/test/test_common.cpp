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
	float test[] = {nanf(""), nanf(""), 1.05, 1.8, 2.0, 2.0, nanf(""), nanf(""), nanf(""), 3.0, nanf(""), nanf("")};
	scan.ranges = std::vector<float>(test, test + sizeof(test)/sizeof(test[0]) - 1);
	//std::cout << "Created test case" << std::endl;

	sensor_msgs::LaserScan corr;
	float corr_res[] = {nanf(""), nanf(""), 1.05, 1.8, 2.0, 2.0, 2.5, 2.5, 2.5, 3.0, nanf(""), nanf("")};
	corr.ranges = std::vector<float>(corr_res, corr_res + sizeof(corr_res)/sizeof(corr_res[0]) - 1);
	//std::cout << "Created correct case" << std::endl;
	common::filterLaserScan(scan, 3);

	for (int i = 0; i < scan.ranges.size(); ++i){
		if (std::isnan(corr.ranges[i])){
			ASSERT_TRUE(std::isnan(scan.ranges[i]));
			continue;
		}
		ASSERT_EQ(corr.ranges[i], scan.ranges[i]);
	}
}

int main(int argc, char **argv){
	ros::init(argc, argv, "test_visualizer", ros::init_options::AnonymousName);
	testing::InitGoogleTest(&argc, argv);

	return RUN_ALL_TESTS();
}
