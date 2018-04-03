/*
 * fsm.cpp
 *
 *  Created on: Mar 27, 2018
 *      Author: kevin
 */

#include "fsm.h"

void FSM::init(State state, std::map<Event, std::map<State, State> > eventMap,
		std::map<State, std::string> stateEmotion, std::string path_to_video) {
	this->eventMap = eventMap;
	this->stateEmotion = stateEmotion;
	this->_path_to_video = path_to_video;
	this->lastTransition = ros::Time::now();
	this->_player = std::make_shared<VideoPlayer>();
	jumpToState(state);
}

State FSM::getCurrentState(){
	return this->currentState;
}

ros::Time FSM::getLastTransitionTime() {
	return this->lastTransition;
}


bool FSM::transition(Event event) {
	State newState = currentState;
	try {
		std::map<State, State> transition = eventMap.at(event);
		newState = transition.at(currentState);
	}catch(const std::exception&e ) {
		//std::cout << "Cannot transition" << std::endl;
		return false;
	}
	jumpToState(newState);
	return true;
}

void FSM::jumpToState(State newState) {
	_player->stop(); //stop the previous video
	std::string emotion = stateEmotion.at(newState);
	std::stringstream ss;
	ss << _path_to_video;
	ss << emotion;
	std::cout << "Entered emotion: " << emotion << std::endl;
	_player->play(ss.str().c_str());
	currentState = newState;
    lastTransition = ros::Time::now();
	return;
}
