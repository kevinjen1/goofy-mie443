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

int main(int argc, char** argv){
	ros::init(argc, argv, "map_navigation_node");
	ros::NodeHandle n;
	ros::spinOnce();
  	teleController eStop;

	ros::Subscriber amclSub = n.subscribe("/amcl_pose", 1, poseCallback);
	
	vector<vector<float> > coord;
	vector<cv::Mat> imgs_track;	
	if(!init(coord, imgs_track)) return 0;

	for(int i = 0; i < coord.size(); ++i){
		cout << i << " x: " << coord[i][0] << " y: " << coord[i][1] << " z: " << coord[i][2] << endl;
	}

	// imageTransporter imgTransport("camera/image/", sensor_msgs::image_encodings::BGR8); // For Webcam
	imageTransporter imgTransport("camera/rgb/image_raw", sensor_msgs::image_encodings::BGR8); //For Kinect

	while(ros::ok()){
		ros::spinOnce();
  		//.....**E-STOP DO NOT TOUCH**.......
   		eStop.block();
    	//...................................

    	//fill with your code

   		// figure out the coordinates of the robot's desired position, given coordinates of boxes
   		// then pass them to moveToGoal
   		bool isMovedToPosition = false;
   		//isMovedToPosition = moveToGoal()
   		if (isMovedToPosition) {
   			ros::Duration(2).sleep(); // wait to ensure robot has settled
   			int foundPic = findPic(imgTransport, imgs_track);
   		}
		
	}
	return 0;
}
