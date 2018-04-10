#include <header.h>
#include <ros/package.h>
#include <imageTransporter.hpp>
#include <opencv2/opencv.hpp>
#include <kobuki_msgs/BumperEvent.h>
#include <kobuki_msgs/CliffEvent.h>
#include <kobuki_msgs/WheelDropEvent.h>
#include "fsm.h"

using namespace std;
using namespace cv;

geometry_msgs::Twist follow_cmd;
bool follow_cmd_status;
int world_state;
FSM fsm;

void followerCB(const geometry_msgs::Twist msg){
    follow_cmd = msg;
}

void followerStatusCB(const std_msgs::Bool::ConstPtr& msg){
    //std::cout << "I'm a follower status!" << std::endl;
    // 1 if the object to follow was detected
    // 0 otherwise (too far, or not enough centroid points)
    if (msg->data) {
    	follow_cmd_status = true;
    	fsm.transition(Event::FollowerPositive);
    } else {
    	follow_cmd_status = false;
   		fsm.transition(Event::FollowerNegative);
    }
}

void bumperCB(const kobuki_msgs::BumperEvent::ConstPtr& msg) {
	if (msg->state == kobuki_msgs::BumperEvent::PRESSED) {
			fsm.transition(Event::Bumper);
	}
}

void wheelCB(const kobuki_msgs::WheelDropEvent::ConstPtr& msg) {
	if (msg->state == kobuki_msgs::WheelDropEvent::RAISED) {
		fsm.transition(Event::Grounded);
	}
	if (msg->state == kobuki_msgs::WheelDropEvent::DROPPED) {
		fsm.transition(Event::Cliff);
	}
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
	ros::Subscriber follower_status = nh.subscribe("turtlebot_follower/follower_found_status", 10, &followerStatusCB);
	ros::Subscriber bumper = nh.subscribe("mobile_base/events/bumper", 10, &bumperCB);
	ros::Subscriber wheel = nh.subscribe("mobile_base/events/wheel_drop", 10, &wheelCB);

	imageTransporter rgbTransport("camera/image/", sensor_msgs::image_encodings::BGR8); //--for Webcam
	//imageTransporter rgbTransport("camera/rgb/image_raw", sensor_msgs::image_encodings::BGR8); //--for turtlebot Camera
	imageTransporter depthTransport("camera/depth_registered/image_raw", sensor_msgs::image_encodings::TYPE_32FC1);

	int world_state = 0;

	double angular = 0.2;
	double linear = 0.0;

	geometry_msgs::Twist vel;
	vel.angular.z = angular;
	vel.linear.x = linear;

	//map between states and names
	std::map<State, std::string> state_emotion;
	state_emotion.insert(std::pair<State, std::string>(State::Static, "blank"));
	state_emotion.insert(std::pair<State, std::string>(State::Discovery, "discovery1")); // happy
	//state_emotion.insert(std::pair<State, std::string>(State::Following, "following"));
	state_emotion.insert(std::pair<State, std::string>(State::Following, "following1")); // extra happy
	state_emotion.insert(std::pair<State, std::string>(State::Obstacle, "obstacle1")); // angry
	state_emotion.insert(std::pair<State, std::string>(State::Lost, "lost1")); // confused //random8
	state_emotion.insert(std::pair<State, std::string>(State::LostExtra, "lost2")); // extra confused //change to obstacle 2
	state_emotion.insert(std::pair<State, std::string>(State::Hanging, "picked1")); // scared
	//state_emotion.insert(std::pair<State, std::string>(State::HangingExtra, "fear2")); // extra scared

	//create list of transitions
	std::map<State, State> trans_vec;

	//map between transitions
	std::map<Event, std::map<State, State> > transition_map;

	trans_vec.clear();
	trans_vec.insert(std::pair<State,State>(State::Obstacle, State::Hanging));
	trans_vec.insert(std::pair<State,State>(State::Discovery, State::Hanging));
	trans_vec.insert(std::pair<State,State>(State::Following, State::Hanging));
	trans_vec.insert(std::pair<State,State>(State::Static, State::Hanging));
	trans_vec.insert(std::pair<State,State>(State::Lost, State::Hanging));
	transition_map.insert(std::pair<Event, std::map<State, State> >(Event::Cliff, trans_vec));

	trans_vec.clear();
	trans_vec.insert(std::pair<State,State>(State::Static, State::Discovery));
	trans_vec.insert(std::pair<State,State>(State::Lost, State::Discovery));
	//trans_vec.insert(std::pair<State,State>(State::Static, State::Following));
	//trans_vec.insert(std::pair<State,State>(State::Lost, State::Following));
	transition_map.insert(std::pair<Event, std::map<State, State> >(Event::FollowerPositive, trans_vec));

	trans_vec.clear();
	trans_vec.insert(std::pair<State,State>(State::Following, State::Lost));
	trans_vec.insert(std::pair<State,State>(State::Discovery, State::Lost));
	transition_map.insert(std::pair<Event, std::map<State, State> >(Event::FollowerNegative, trans_vec));

	trans_vec.clear();
	trans_vec.insert(std::pair<State,State>(State::Obstacle, State::Static));
	trans_vec.insert(std::pair<State,State>(State::Lost, State::LostExtra));
	trans_vec.insert(std::pair<State,State>(State::LostExtra, State::Static));
	trans_vec.insert(std::pair<State,State>(State::Discovery, State::Following));
	//trans_vec.insert(std::pair<State,State>(State::Hanging, State::HangingExtra));
	transition_map.insert(std::pair<Event, std::map<State, State> >(Event::Timeout, trans_vec));

	trans_vec.clear();
	trans_vec.insert(std::pair<State,State>(State::Following, State::Obstacle));
	trans_vec.insert(std::pair<State,State>(State::Discovery, State::Obstacle));
	transition_map.insert(std::pair<Event, std::map<State, State> >(Event::Bumper, trans_vec));

	trans_vec.clear();
	trans_vec.insert(std::pair<State,State>(State::Hanging, State::Static));
	//trans_vec.insert(std::pair<State,State>(State::HangingExtra, State::Static));
	transition_map.insert(std::pair<Event, std::map<State, State> >(Event::Grounded, trans_vec));

	fsm.init(State::Static, transition_map, state_emotion, path_to_videos);

	//map between states and timeouts
	std::map<State, ros::Duration> state_timeout;
	state_timeout.insert(std::pair<State, ros::Duration>(State::Discovery, ros::Duration(3)));
	state_timeout.insert(std::pair<State, ros::Duration>(State::Obstacle, ros::Duration(7)));
	state_timeout.insert(std::pair<State, ros::Duration>(State::Lost, ros::Duration(3)));
	state_timeout.insert(std::pair<State, ros::Duration>(State::LostExtra, ros::Duration(3)));
	//state_timeout.insert(std::pair<State, ros::Duration>(State::Hanging, ros::Duration(3)));
    
    std::cout << "Finished initializations" << std::endl;
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
			case State::Static: {
				geometry_msgs::Twist motion;
				motion.linear.x = 0;
				motion.linear.y = 0;
				motion.linear.z = 0;
				motion.angular.x = 0;
				motion.angular.y = 0;
				motion.angular.z = 0;
				vel_pub.publish(motion);
				break;
			}
			case State::Discovery: {
				vel_pub.publish(follow_cmd);
				break;
			}
			case State::Following: {
				vel_pub.publish(follow_cmd);
				break;
			}
			case State::Obstacle: {
				// try to have a range of motion, where it stops, backs up and tries to go again
				ros::Duration rosDuration = ros::Time::now() - fsm.getLastTransitionTime();
				double duration = rosDuration.toSec();
				geometry_msgs::Twist motion;
				if (duration < 0.5){
					motion.linear.x = 0;
					motion.linear.y = 0;
					motion.linear.z = 0;
					motion.angular.x = 0;
					motion.angular.y = 0;
					motion.angular.z = 0;
				} else if (duration < 1.5) {
					motion.linear.x = -0.3;
					motion.linear.y = 0;
					motion.linear.z = 0;
					motion.angular.x = 0;
					motion.angular.y = 0;
					motion.angular.z = 0;
				} else if (duration < 2.5) {
					motion.linear.x = 0.3;
					motion.linear.y = 0;
					motion.linear.z = 0;
					motion.angular.x = 0;
					motion.angular.y = 0;
					motion.angular.z = 0;
				} else if (duration < 3.5) {
					motion.linear.x = -0.6;
					motion.linear.y = 0;
					motion.linear.z = 0;
					motion.angular.x = 0;
					motion.angular.y = 0;
					motion.angular.z = 0;
				} else if (duration < 4.5) {
					motion.linear.x = 0.6;
					motion.linear.y = 0;
					motion.linear.z = 0;
					motion.angular.x = 0;
					motion.angular.y = 0;
					motion.angular.z = 0;
				} else if (duration < 6.0) {
					motion.linear.x = 0;
					motion.linear.y = 0;
					motion.linear.z = 0;
					motion.angular.x = 0;
					motion.angular.y = 0;
					motion.angular.z = 0;
				}
                vel_pub.publish(motion);
				break;
			}
			case State::Lost: {
			    ros::Duration rosDuration = ros::Time::now() - fsm.getLastTransitionTime();
				double duration = rosDuration.toSec();
				geometry_msgs::Twist motion;
				if (duration < 0.5){
					motion.linear.x = 0;
					motion.linear.y = 0;
					motion.linear.z = 0;
					motion.angular.x = 0;
					motion.angular.y = 0;
					motion.angular.z = 0.7;
				} else if (duration < 1.5) {
					motion.linear.x = 0;
					motion.linear.y = 0;
					motion.linear.z = 0;
					motion.angular.x = 0;
					motion.angular.y = 0;
					motion.angular.z = -0.7;
				}
				vel_pub.publish(motion);
				break;
			}
			case State::LostExtra: {
				// confused, turn in place
				geometry_msgs::Twist motion;
				motion.angular.z = 2.5;
				vel_pub.publish(motion);
				break;
			}
			case State::Hanging: {
				// wants to escape. keep turning in place
				geometry_msgs::Twist motion;
				motion.linear.x = 0;
				motion.linear.y = 0;
				motion.linear.z = 0;
				motion.angular.x = 0;
				motion.angular.y = 0;
				motion.angular.z = 2;
				//vel_pub.publish(motion);
				break;
			}
			default: {
				break;
			}
		}
		auto loc = state_timeout.find(fsm.getCurrentState());
		if (loc != state_timeout.end()){
		    ros::Duration diff = ros::Time::now() - fsm.getLastTransitionTime();
			if (diff > (*loc).second){
			    std::cout << "Timeout transition: " << diff << std::endl;
				fsm.transition(Event::Timeout);
			}
		}

	}

	return 0;
}
