#include <header.h>
#include <ros/package.h>
#include <imageTransporter.hpp>
#include <opencv2/opencv.hpp>
#include "fsm.h"

using namespace std;
using namespace cv;

geometry_msgs::Twist follow_cmd;
int world_state;

void followerCB(const geometry_msgs::Twist msg){
    follow_cmd = msg;
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

	FSM fsm;

	while(ros::ok()){
		ros::spinOnce();
		//.....**E-STOP DO NOT TOUCH**.......
		//eStop.block();
		//...................................

		//add state transitions based on callbacks here

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
		case (Obstacle):{
			break;
		}
		default: {
			break;
		}
		}
	}

	return 0;
}
