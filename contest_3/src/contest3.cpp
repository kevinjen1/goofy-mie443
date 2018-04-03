#include <header.h>
#include <ros/package.h>
#include <imageTransporter.hpp>
#include <opencv2/opencv.hpp>
#include <kobuki_msgs/BumperEvent.h>
#include <kobuki_msgs/CliffEvent.h>
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
    // 1 if the object to follow was detected
    // 0 otherwise (too far, or not enough centroid points)
    if (msg->data) {
    	follow_cmd_status = true;
    	if (fsm.getCurrentState() == State::Static) {
    		fsm.transition(Event::FollowerPositive);
    	}
    } else {
    	follow_cmd_status = false;
    	if (fsm.getCurrentState() == State::Following) {
    		fsm.transition(Event::FollowerNegative);
    	}
    }
}

void bumperCB(const kobuki_msgs::BumperEvent::ConstPtr& msg) {
	if (msg->state == kobuki_msgs::BumperEvent::PRESSED) {
		if (fsm.getCurrentState() == State::Obstacle) {
			fsm.transition(Event::Bumper);
		}
	}
}

void cliffCB(const kobuki_msgs::CliffEvent::ConstPtr& msg) {
	if (fsm.getCurrentState() == State::Hanging) {
		if (msg->state == kobuki_msgs::CliffEvent::FLOOR) {
			fsm.transition(Event::Grounded);
		}
	} else {
		if (msg->state == kobuki_msgs::CliffEvent::CLIFF) {
			fsm.transition(Event::Cliff);
		}
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
	ros::Subscriber follower_status = nh.subscribe("follower_found_status", 10, &followerStatusCB);
	ros::Subscriber bumper = nh.subscribe("mobile_base/events/bumper", 10, &bumperCB);
	ros::Subscriber cliff = nh.subscribe("mobile_base/events/cliff", 10, &cliffCB);

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

	//map between states and names
	std::map<State, std::string> state_emotion;
	state_emotion.insert(std::pair<State, std::string>(State::Static, "static"));
	state_emotion.insert(std::pair<State, std::string>(State::Discovery, "discovery"));
	state_emotion.insert(std::pair<State, std::string>(State::Following, "following"));
	state_emotion.insert(std::pair<State, std::string>(State::Obstacle, "obstacle"));
	state_emotion.insert(std::pair<State, std::string>(State::Lost, "lost"));
	state_emotion.insert(std::pair<State, std::string>(State::Hanging, "hanging"));

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
	transition_map.insert(std::pair<Event, std::map<State, State> >(Event::FollowerPositive, trans_vec));

	trans_vec.clear();
	trans_vec.insert(std::pair<State,State>(State::Following, State::Lost));
	transition_map.insert(std::pair<Event, std::map<State, State> >(Event::FollowerNegative, trans_vec));

	trans_vec.clear();
	trans_vec.insert(std::pair<State,State>(State::Obstacle, State::Static));
	trans_vec.insert(std::pair<State,State>(State::Lost, State::Static));
	trans_vec.insert(std::pair<State,State>(State::Discovery, State::Following));
	transition_map.insert(std::pair<Event, std::map<State, State> >(Event::Timeout, trans_vec));

	trans_vec.clear();
	trans_vec.insert(std::pair<State,State>(State::Following, State::Obstacle));
	transition_map.insert(std::pair<Event, std::map<State, State> >(Event::Bumper, trans_vec));

	trans_vec.clear();
	trans_vec.insert(std::pair<State,State>(State::Hanging, State::Static));
	transition_map.insert(std::pair<Event, std::map<State, State> >(Event::Grounded, trans_vec));

	fsm.init(State::Static, transition_map, state_emotion, path_to_videos);

	//map between states and timeouts
	std::map<State, ros::Duration> state_timeout;
	state_timeout.insert(std::pair<State, ros::Duration>(State::Discovery, ros::Duration(1)));
	state_timeout.insert(std::pair<State, ros::Duration>(State::Obstacle, ros::Duration(5)));
	state_timeout.insert(std::pair<State, ros::Duration>(State::Lost, ros::Duration(4)));

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
				if (duration < 500){
					geometry_msgs::Twist motion;
					motion.linear.x = 0;
					motion.linear.y = 0;
					motion.linear.z = 0;
					motion.angular.x = 0;
					motion.angular.y = 0;
					motion.angular.z = 0;
				} else if (duration < 1500) {
					geometry_msgs::Twist motion;
					motion.linear.x = -0.5;
					motion.linear.y = 0;
					motion.linear.z = 0;
					motion.angular.x = 0;
					motion.angular.y = 0;
					motion.angular.z = 0;
				} else if (duration < 2500) {
					geometry_msgs::Twist motion;
					motion.linear.x = 0.5;
					motion.linear.y = 0;
					motion.linear.z = 0;
					motion.angular.x = 0;
					motion.angular.y = 0;
					motion.angular.z = 0;
				} else if (duration < 3500) {
					geometry_msgs::Twist motion;
					motion.linear.x = -0.8;
					motion.linear.y = 0;
					motion.linear.z = 0;
					motion.angular.x = 0;
					motion.angular.y = 0;
					motion.angular.z = 0;
				} else if (duration < 4500) {
					geometry_msgs::Twist motion;
					motion.linear.x = 0.8;
					motion.linear.y = 0;
					motion.linear.z = 0;
					motion.angular.x = 0;
					motion.angular.y = 0;
					motion.angular.z = 0;
				} else if (duration < 5500) {
					geometry_msgs::Twist motion;
					motion.linear.x = 0;
					motion.linear.y = 0;
					motion.linear.z = 0;
					motion.angular.x = 0;
					motion.angular.y = 0;
					motion.angular.z = 0;
				}

				break;
			}
			case State::Lost: {
				// happy, turn in place
				geometry_msgs::Twist motion;
				motion.angular.z = 0.1;
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
				motion.angular.z = 0.5;
				break;
			}
			default: {
				break;
			}
		}
		auto loc = state_timeout.find(fsm.getCurrentState());
		if (loc != state_timeout.end()){
			if (ros::Time::now() - fsm.getLastTransitionTime() > (*loc).second){
				fsm.transition(Event::Timeout);
			}
		}

	}

	return 0;
}
