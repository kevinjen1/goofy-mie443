/*
 * fsm.cpp
 *
 *  Created on: Mar 27, 2018
 *      Author: kevin
 */

#include "fsm.h"

FSM::FSM(State state, std::map<Event, std::vector<Transition> > eventMap, std::map<State, std::string> stateEmotion) {
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
	return false;
}

void jumpToState(enum State) {
}
