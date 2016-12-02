//behavior_FSM.h

#ifndef BEHAVIOR_FSM_H_
#define BEHAVIOR_FSM_H_

#include "init.h"

void find_state( dd *robot, pk *puck);

#endif /* BEHAVIOR_FSM_H_ */



// typedef struct {
// 	state st;
// 	event ev;
// 	state_fn * next;
// }tTransition;
// tTransition transArr[] = {
// 	{ST_PK_SEARCH,	EV_PK_OBSCURED,	&puck_search},
// 	{ST_PK_SEARCH,	EV_PK_FOUND,	&puck_pursue},
// 	{ST_PK_PURSUE,	EV_PK_FOUND,	&puck_pursue},
// 	{ST_PK_PURSUE,	EV_PK_OBSCURED,	&puck_search},
// 	{ST_PK_PURSUE,	EV_PK_OBTAINED,	&puck_to_goal},
// 	{ST_PK_TO_GOAL,	EV_PK_OBTAINED,	&puck_to_goal},
// 	{ST_PK_TO_GOAL,	EV_PK_LOST,		&puck_pursue},
// 	{ST_PK_TO_GOAL,	EV_GOAL_MADE,	&goal_made},
// 	{ST_ERROR,		EV_ERROR,		&fsm_error};

// };
// struct state;

// typedef void state_fn(struct state *);
// struct state
// {	
	
//     state_fn * next;
//     int i; // data
// };


// void find_state( dd *robot, pk *puck);
