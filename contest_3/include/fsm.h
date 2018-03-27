/*
 * fsm.h
 *
 *  Created on: Mar 15, 2018
 *      Author: turtlebot
 */

#ifndef GOOFY_MIE443_CONTEST_3_INCLUDE_FSM_H_
#define GOOFY_MIE443_CONTEST_3_INCLUDE_FSM_H_

#include <map>
#include <string>
#include "ros/ros.h"

enum State {
	Static,
	Discovey,
	Following,
	Obstacle,
	Lost,
	Hanging
};

enum Event {
	Bumper,
	Cliff,
	Grounded,
	FollowerPositive,
	FollowerNegative,
	GoStatic
};

struct Transition {
	State oldState;
	State endState;
};

class FSM {
private:
	State currentState;
	ros::Time lastTransition;
	std::map<Event, Transition[]> eventMap;
	std::map<State, std::string> stateEmotion;

public:
	bool transition(enum Event);
	State getCurrentState();
	void jumpToState(enum State);
	ros::Time getLastTransitionTime();

};




#endif /* GOOFY_MIE443_CONTEST_3_INCLUDE_FSM_H_ */
