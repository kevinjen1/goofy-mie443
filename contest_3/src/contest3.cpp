#include <header.h>
#include <ros/package.h>
#include <imageTransporter.hpp>
#include "fsm.h"

using namespace std;

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
