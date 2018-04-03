/*c
 * fsm.h
 *
 *  Created on: Mar 15, 2018
 *      Author: turtlebot
 */

#ifndef GOOFY_MIE443_CONTEST_3_INCLUDE_FSM_H_
#define GOOFY_MIE443_CONTEST_3_INCLUDE_FSM_H_

#include <map>
#include <string>
#include <memory>
#include "ros/ros.h"
#include"PlayVideo.hpp"

enum class State {
	Static,
	Discovery,
	Following,
	Obstacle,
	Lost,
	Hanging,
	HangingExtra,
	LostExtra
};

enum class Event {
	Bumper,
	Cliff,
	Grounded,
	FollowerPositive,
	FollowerNegative,
	Timeout
};

class FSM {
private:
	State currentState;
	ros::Time lastTransition;
	std::map<Event, std::map<State, State> > eventMap;
	std::map<State, std::string> stateEmotion;

	std::string _path_to_video;
	std::shared_ptr<VideoPlayer> _player;

public:
	void init(State state, std::map<Event, std::map<State,State> > eventMap,
			std::map<State, std::string> stateEmotion,
			std::string path_to_video);
	bool transition(enum Event);
	State getCurrentState();
	void jumpToState(enum State);
	ros::Time getLastTransitionTime();

};




#endif /* GOOFY_MIE443_CONTEST_3_INCLUDE_FSM_H_ */
