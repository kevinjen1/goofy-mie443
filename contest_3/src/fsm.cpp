/*
 * fsm.cpp
 *
 *  Created on: Mar 27, 2018
 *      Author: kevin
 */

#include "fsm.h"

FSM::FSM(State state, std::map<Event, std::map<State, State> > eventMap, std::map<State, std::string> stateEmotion) {
	this->currentState = currentState;
	this->eventMap = eventMap;
	this->stateEmotion = stateEmotion;
}

State FSM::getCurrentState(){
	return this->currentState;
}

ros::Time FSM::getLastTransitionTime() {
	return this->lastTransition;
}


bool FSM::transition(Event event) {
	try {
		std::map<State, State> transition = this->eventMap.at(event);
		State newState = transition.at(this->currentState);
		jumpToState(newState);
		return true;
	}catch(const std::exception&e ) {
		return false;
	}
}

void FSM::jumpToState(enum State) {
	return;
}
