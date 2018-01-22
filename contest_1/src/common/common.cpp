#include"common/common.hpp"

namespace goofy{
namespace common{

nav_msgs::Path RobotModel::simulatePath(double lin_vel, double ang_vel, double time, double density){
	int num_intervals = std::floor(lin_vel * time / density);
	double time_per_interval = time/num_intervals;

	geometry_msgs::PoseStamped base_pose;
	tf::Quaternion
	base_pose.header.frame_id = common::BASE;

	base_pose.pose.position.x = 0;
	base_pose.pose.position.y = 0;
	base_pose.pose.position.z = 0;

	nav_msgs::Path retval;
	retval.header.frame_id = common::BASE;

	for i = 1:num_intervals;
}

}
}

