#include <header.h>
#include <ros/package.h>
#include <imageTransporter.hpp>
#include <opencv2/opencv.hpp>
#include "fsm.h"

using namespace std;
using namespace cv;

geometry_msgs::Twist follow_cmd;
int follow_cmd_status;
int world_state;

void followerCB(const geometry_msgs::Twist msg){
    follow_cmd = msg;
}

void followerStatusCB(const std_msgs::Bool::ConstPtr& msg){
    // 1 if the object to follow was detected
    // 0 otherwise (too far, or not enough centroid points)
    if (msg->data) { follow_cmd_status = 1;}
    else { follow_cmd_status = 0;}
}

void bumperCB(const geometry_msgs::Twist msg){
    //Fill with code
}

//-------------------------------------------------------------

int main(int argc, char **argv)
{
	ros::init(argc, argv, "image_listener");
	ros::NodeHandle nh;
	sound_play::SoundClient sc;
	string path_to_sounds = ros::package::getPath("mie443_contest3") + "/sounds/";
	string path_to_videos = ros::package::getPath("mie443_contest3") + "/videos/";
	teleController eStop;

	//publishers
	ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop",1);

	//subscribers
	ros::Subscriber follower = nh.subscribe("follower_velocity_smoother/smooth_cmd_vel", 10, &followerCB);
	//ros::Subscriber follower_status = nh.subscribe("follower_found_status", 10, &followerStatusCB);
	ros::Subscriber bumper = nh.subscribe("mobile_base/events/bumper", 10, &bumperCB);

	imageTransporter rgbTransport("camera/image/", sensor_msgs::image_encodings::BGR8); //--for Webcam
	//imageTransporter rgbTransport("camera/rgb/image_raw", sensor_msgs::image_encodings::BGR8); //--for turtlebot Camera
	imageTransporter depthTransport("camera/depth_registered/image_raw", sensor_msgs::image_encodings::TYPE_32FC1);

	int world_state = 0;

	double angular = 0.2;
	double linear = 0.0;

	geometry_msgs::Twist vel;
	vel.angular.z = angular;
	vel.linear.x = linear;

	sc.playWave(path_to_sounds + "sound.wav");
	ros::Duration(0.5).sleep();
	
	//-- GRACE - VIEDO + SOUND
	/*
	sc.playWave(path_to_sounds + "shocked3.wav");
	
	VideoCapture cap(path_to_videos + "shocked3.mp4"); 
    
  // Check if camera opened successfully
  if(!cap.isOpened()){
    cout << "Error opening video stream or file" << endl;
    return -1;
  }
  //sc.playWave(path_to_sounds + "shocked3.wav");
  while(1){
 
    Mat frame;
    // Capture frame-by-frame
    cap >> frame;
  
    // If the frame is empty, break immediately
    if (frame.empty())
      break;
 
    // Display the resulting frame
    imshow( "Frame", frame );
 
    // Press  ESC on keyboard to exit
    char c=(char)waitKey(25);
    if(c==27)
      break;
  }
  
  // When everything done, release the video capture object
  cap.release();
 
  // Closes all the frames
  destroyAllWindows(); */
  // GRACE -- VIDEO AND SOUND

	//map between states and names
	std::map<State, std::string> state_emotion;
	state_emotion.insert(std::pair<State, std::string>(Static, "static"));
	state_emotion.insert(std::pair<State, std::string>(Discovery, "discovery"));
	state_emotion.insert(std::pair<State, std::string>(Following, "following"));
	state_emotion.insert(std::pair<State, std::string>(Obstacle, "obstacle"));
	state_emotion.insert(std::pair<State, std::string>(Lost, "lost"));
	state_emotion.insert(std::pair<State, std::string>(Hanging, "hanging"));

	//create list of transitions
	std::map<State, State> trans_vec;

	//map between transitions
	std::map<Event, std::map<State, State> > transition_map;

	trans_vec.clear();
	trans_vec.insert(std::pair<State,State>(Obstacle, Hanging));
	trans_vec.insert(std::pair<State,State>(Discovery, Hanging));
	trans_vec.insert(std::pair<State,State>(Following, Hanging));
	trans_vec.insert(std::pair<State,State>(Static, Hanging));
	trans_vec.insert(std::pair<State,State>(Lost, Hanging));
	transition_map.insert(std::pair<Event, std::map<State, State> >(Cliff, trans_vec));

	trans_vec.clear();
	trans_vec.insert(std::pair<State,State>(Static, Discovery));
	transition_map.insert(std::pair<Event, std::map<State, State> >(FollowerPositive, trans_vec));

	trans_vec.clear();
	trans_vec.insert(std::pair<State,State>(Following, Lost));
	transition_map.insert(std::pair<Event, std::map<State, State> >(FollowerNegative, trans_vec));

	trans_vec.clear();
	trans_vec.insert(std::pair<State,State>(Obstacle, Static));
	trans_vec.insert(std::pair<State,State>(Lost, Static));
	trans_vec.insert(std::pair<State,State>(Discovery, Following));
	transition_map.insert(std::pair<Event, std::map<State, State> >(Timeout, trans_vec));

	trans_vec.clear();
	trans_vec.insert(std::pair<State,State>(Following, Obstacle));
	transition_map.insert(std::pair<Event, std::map<State, State> >(Bumper, trans_vec));

	trans_vec.clear();
	trans_vec.insert(std::pair<State,State>(Hanging, Static));
	transition_map.insert(std::pair<Event, std::map<State, State> >(Grounded, trans_vec));

	FSM fsm(Static, transition_map, state_emotion);

	//map between states and timeouts
	std::map<State, ros::Duration> state_timeout;
	state_timeout.insert(std::pair<State, ros::Duration>(Discovery, ros::Duration(1)));
	state_timeout.insert(std::pair<State, ros::Duration>(Obstacle, ros::Duration(4)));
	state_timeout.insert(std::pair<State, ros::Duration>(Lost, ros::Duration(4)));

	while(ros::ok()){
		ros::spinOnce();
		//.....**E-STOP DO NOT TOUCH**.......
		//eStop.block();
		//...................................

		//add state transitions based on callbacks here

		/*
		// Check follower output to see if it lost sight of the person.
		// Not sure where in the workflow you want this, so leaving it here and commented out for now
		if (follow_cmd_status) {
		   // Change states
		   // Might require some logic in terms of "if last x were 0, then call it quits"
		}
		
		*/

		//manage motions from here
		switch (fsm.getCurrentState()){
			case (Discovery):{
				vel_pub.publish(follow_cmd);
				break;
			}
			case (Following):{
				vel_pub.publish(follow_cmd);
				break;
			}
			default: {
				break;
			}
		}
		auto loc = state_timeout.find(fsm.getCurrentState());
		if (loc != state_timeout.end()){
			if (ros::Time::now() - fsm.getLastTransitionTime() > (*loc).second){
				fsm.transition(Timeout);
			}
		}

	}

	return 0;
}
