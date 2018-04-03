/*
 * test_threading.cpp
 *
 *  Created on: Apr 3, 2018
 *      Author: kevin
 */

#include<gtest/gtest.h>
#include<ros/ros.h>
#include<chrono>
#include<thread>
#include <ros/package.h>
#include"PlayVideo.hpp"
#include"fsm.h"

std::string ROOT_FILE = ROOT_DIR;

TEST(ThreadingTest, MainTest){
	VideoPlayer video_player;
	std::stringstream ss;
	ss << ROOT_FILE;
	ss << "/videos/shocked3";
	std::cout << "Reading " << ss.str() << std::endl;
	video_player.play(ss.str().c_str());
	std::cout << "Video started" << std::endl;
	std::this_thread::sleep_for(std::chrono::milliseconds(2000));
}

TEST(ThreadingTest, JoinBeforeCreate){
	VideoPlayer video_player;
	video_player.stop();
}

TEST(FSMTest, Transition){
	FSM fsm;

	std::map<Event, std::map<State, State> > transition_map;
	std::string path_to_videos = ros::package::getPath("mie443_contest3") + "/videos/";

	std::map<State, State> trans_vec;
	trans_vec.clear();
	trans_vec.insert(std::pair<State,State>(State::Static, State::Hanging));
	transition_map.insert(std::pair<Event, std::map<State, State> >(Event::Cliff, trans_vec));

	std::map<State, std::string> state_emotion;
	state_emotion.insert(std::pair<State, std::string>(State::Static, "static"));
	state_emotion.insert(std::pair<State, std::string>(State::Hanging, "shocked3"));

	fsm.init(State::Static, transition_map, state_emotion, path_to_videos);

	fsm.transition(Event::Cliff);
	std::this_thread::sleep_for(std::chrono::milliseconds(5000));
}

int main(int argc, char **argv){
	ros::init(argc, argv, "test_visualizer", ros::init_options::AnonymousName);
	testing::InitGoogleTest(&argc, argv);

	return RUN_ALL_TESTS();
}

