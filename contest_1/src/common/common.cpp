#include"common/common.hpp"

namespace goofy{
namespace common{

nav_msgs::Path RobotModel::simulatePath(double lin_vel, double ang_vel, double time, double density){
	int num_intervals = std::floor(lin_vel * time / density);
	double time_per_interval = time/num_intervals;

	geometry_msgs::PoseStamped base_pose;
	base_pose.header.frame_id = common::BASE;
	double cur_yaw = 0;
	base_pose.pose.position.x = 0;
	base_pose.pose.position.y = 0;
	base_pose.pose.position.z = 0;

	nav_msgs::Path retval;
	common::PoseArray pose_array;
	retval.header.frame_id = common::BASE;

	for (int i = 0; i < num_intervals; ++i){
		//add half the yaw
		cur_yaw += time_per_interval * ang_vel/2;

		//augment the position
		base_pose.pose.position.x += lin_vel * cos(cur_yaw) * time_per_interval;
		base_pose.pose.position.y += lin_vel * sin(cur_yaw) * time_per_interval;

		//add the remaining yaw
		cur_yaw += time_per_interval * ang_vel/2;

		//convert the yaw into a quaternion
		base_pose.pose.orientation = yaw2quat(cur_yaw);

		//add the pose to the vector
		pose_array.push_back(base_pose);
	}
	retval.poses = pose_array;

	//todo: due to the flooring of the num_intervals, the path is cut short
	return retval;
}

geometry_msgs::Quaternion yaw2quat(double yaw){
	tf::Quaternion quaternion;
	geometry_msgs::Quaternion retval;

	quaternion.setEuler(yaw, 0, 0);
	retval.x = quaternion.getX();
	retval.y = quaternion.getY();
	retval.z = quaternion.getZ();
	retval.w = quaternion.getW();

	return retval;
}

}
}

