#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <func_header.h>

#include <eStop.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

float x;
float y;
float phi;

void poseCallback(const geometry_msgs::PoseWithCovarianceStamped& msg){
	phi = tf::getYaw(msg.pose.pose.orientation);
    	x = msg.pose.pose.position.x;
    	y = msg.pose.pose.position.y;
}

//-------------------------move robot function---------------
bool moveToGoal(float xGoal, float yGoal, float phiGoal){

	//define a client for to send goal requests to the move_base server through a SimpleActionClient
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

	//wait for the action server to come up
	while(!ac.waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the move_base action server to come up");
	}

	move_base_msgs::MoveBaseGoal goal;

	//set up the frame parameters
	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();

	/* moving towards the goal*/
    	geometry_msgs::Quaternion phi = tf::createQuaternionMsgFromYaw(phiGoal);

	goal.target_pose.pose.position.x =  xGoal;
	goal.target_pose.pose.position.y =  yGoal;
	goal.target_pose.pose.position.z =  0.0;
	goal.target_pose.pose.orientation.x = 0.0;
	goal.target_pose.pose.orientation.y = 0.0;
	goal.target_pose.pose.orientation.z = phi.z;
	goal.target_pose.pose.orientation.w = phi.w;

	ROS_INFO("Sending goal location ...");
	ac.sendGoal(goal);

	ac.waitForResult();

	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
		ROS_INFO("You have reached the destination");
		return true;
	}
	else{
		ROS_INFO("The robot failed to reach the destination");
		return false;
	}
}

Cereal::Cereal (int one, int two) {
	coord = one;
	logo = two;
}

Status::Status(int one, bool two) {
	coord = one;
	success = two;
}

/**
 * Change the variable 'coordIndex' to point an explored position
 * Returns true if unexplored position was found. Otherwise false
 */
bool getNextCoord(int* coordIndex, int count, vector<Status> mission) {
	int startingIndex = *coordIndex;
	int index = startingIndex;

	do {
		index = (index+1)%count;
	} while (index != startingIndex || !mission[index].success);

	if (index == startingIndex) {
		if (mission[index].success) {
			return false; // successfully completed all missions
		}
	}
	*coordIndex = index;
	return true;
}

int main(int argc, char** argv){
	ros::init(argc, argv, "map_navigation_node");
	ros::NodeHandle n;
	ros::spinOnce();
  	teleController eStop;

	ros::Subscriber amclSub = n.subscribe("/amcl_pose", 1, poseCallback);
	
	vector<vector<float> > coord;
	vector<cv::Mat> imgs_track;	
	vector<Cereal> mappedPics;
	vector<Status> mission;

	if(!init(coord, imgs_track)) return 0;

	int count = coord.size();

	for(int i = 0; i < count; ++i){
		cout << i << " x: " << coord[i][0] << " y: " << coord[i][1] << " z: " << coord[i][2] << endl;
		mission.push_back(Status(i,false));
	}

	// imageTransporter imgTransport("camera/image/", sensor_msgs::image_encodings::BGR8); // For Webcam
	imageTransporter imgTransport("camera/rgb/image_raw", sensor_msgs::image_encodings::BGR8); //For Kinect

	int coordIndex = 0;

	while(ros::ok()){
		ros::spinOnce();
  		//.....**E-STOP DO NOT TOUCH**.......
   		eStop.block();
    	//...................................

    	//fill with your code

   		bool isMovedToPosition = false;
   		// figure out the coordinates of the robot's desired position, given coordinates of boxes (coord[coordIndex])
   		// then pass them to moveToGoal
   		//isMovedToPosition = moveToGoal()

   		if (isMovedToPosition) {
   			ros::Duration(2).sleep(); // wait to ensure robot has settled
   			int foundPic = findPic(imgTransport, imgs_track);
   			if (foundPic >= 0) {
   				mappedPics.push_back(Cereal (coordIndex, foundPic));
   				mission[coordIndex].success = true;
   			}
   		} else {
   			//figure out something
   		}
   		bool isMoreToGo = getNextCoord(&coordIndex, count, mission);

   		if (!isMoreToGo) {
   			// display logo for each coordinate
   			// go back to beginning. Sing a lullaby. Do a victory dance.
   			break;
   		}

	}
	return 0;
}
