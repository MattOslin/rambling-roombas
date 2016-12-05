//behavior_FSM.c
//Controls the state of the system

#include <stdio.h>
#include "behavior_FSM.h"

//http://stackoverflow.com/questions/1647631/c-state-machine-design/1647679#1647679

//ONE OPTION
//Default state must be ZERO
typedef enum {ST_ERROR = -1, ST_STANDBY, ST_PK_SEARCH, ST_PK_BEHIND, ST_PK_PURSUE, ST_PK_TO_GOAL, ST_GOAL_MADE, NUM_ST} state;
typedef enum {EV_ERROR = -1, EV_STANDBY, EV_PLAY, EV_PK_OBSCURED, EV_PK_FOUND, EV_PK_BEHIND, EV_PK_OBTAINED, EV_GOAL_MADE, NUM_EV} event;

typedef state state_fn(dd *robot, pk *puck);

state_fn fsm_error, standby, puck_search, puck_pursue, puck_to_goal, goal_made, puck_behind,puck_behind_persist;

static state_fn *transArr[NUM_ST][NUM_EV];

void init_fsm(){

	int i,j;
	for (i = 0; i < NUM_ST; i++) {
		for (j = 0; j < NUM_EV; j++) {
	    	transArr[i][j] = &fsm_error;
		}
	}
	transArr[ST_PK_SEARCH]	[EV_PK_OBSCURED]	= &puck_search;
	transArr[ST_PK_SEARCH]	[EV_PK_FOUND]		= &puck_pursue;
	transArr[ST_PK_SEARCH]	[EV_PK_BEHIND]		= &puck_behind;

	transArr[ST_PK_BEHIND]	[EV_PK_FOUND]		= &puck_pursue;
	transArr[ST_PK_BEHIND]	[EV_PK_BEHIND]		= &puck_behind_persist;
	transArr[ST_PK_BEHIND]	[EV_PK_OBSCURED]	= &puck_search;

	transArr[ST_PK_PURSUE]	[EV_PK_FOUND]		= &puck_pursue;
	transArr[ST_PK_PURSUE]	[EV_PK_OBSCURED]	= &puck_search;
	transArr[ST_PK_PURSUE]	[EV_PK_BEHIND]		= &puck_behind;
	transArr[ST_PK_PURSUE]	[EV_PK_OBTAINED]	= &puck_to_goal;

	transArr[ST_PK_TO_GOAL]	[EV_PK_OBTAINED]	= &puck_to_goal;
	transArr[ST_PK_TO_GOAL]	[EV_PK_FOUND]		= &puck_pursue;
	transArr[ST_PK_TO_GOAL]	[EV_PK_OBSCURED]	= &puck_search;
	transArr[ST_PK_TO_GOAL]	[EV_PK_BEHIND]		= &puck_search;
	transArr[ST_PK_TO_GOAL]	[EV_GOAL_MADE]		= &goal_made;

	transArr[ST_GOAL_MADE]	[EV_GOAL_MADE]		= &goal_made;
	transArr[ST_GOAL_MADE]	[EV_PK_OBSCURED]	= &puck_search;
	transArr[ST_GOAL_MADE]	[EV_PK_FOUND]		= &puck_pursue;
	transArr[ST_GOAL_MADE]	[EV_PK_BEHIND]		= &puck_behind;
	transArr[ST_GOAL_MADE]	[EV_PK_OBTAINED]	= &puck_to_goal;

	for (i = 0; i < NUM_ST; i++) {
	    	transArr[i][EV_STANDBY] = &standby;
	}

	transArr[ST_STANDBY]    [EV_GOAL_MADE]        = &goal_made;
    transArr[ST_STANDBY]    [EV_PLAY]            = &puck_search;
    transArr[ST_STANDBY]    [EV_PK_OBSCURED]    = &puck_search;
    transArr[ST_STANDBY]    [EV_PK_FOUND]        = &puck_pursue;
    transArr[ST_STANDBY]    [EV_PK_BEHIND]        = &puck_behind;
    transArr[ST_STANDBY]    [EV_PK_OBTAINED]    = &puck_to_goal;
}

//BREAD AND BUTTER OF STATE MACHINE

void event_handler_fsm( dd *robot, pk *puck ){
	event ev = robot->ev;
	if(robot->enable){
		if (robot->goalMade){
			robot->goalMade = 0;
			robot->ev = EV_GOAL_MADE;
			return;
		}
		if (robot->ev == EV_GOAL_MADE){
			if(dd_is_loc(robot,5,.1)){
				robot->ev = EV_STANDBY;
				dd_disable(robot);
			}
			return;
		}

		if(puck->isFound){
			if(puck->isBehind){
				ev = EV_PK_BEHIND;
			}
			else if(puck->isHave){
				ev = EV_PK_OBTAINED;
			}
			else{
				ev = EV_PK_FOUND;
			}
		}
		else {
			if (robot->ev == EV_PK_BEHIND && !dd_is_loc(robot,5,.1)){
				return;
			}
			else{
				ev = EV_PK_OBSCURED;
			}

		}
		robot->ev = ev;
	}
	else
	{
		robot->ev = EV_STANDBY;
	}
	
}


void find_state( dd *robot, pk *puck)
{
	event_handler_fsm(robot,puck);
	// printf("%i\t",robot->ev);
	robot->nxtSt = transArr[robot->nxtSt][robot->ev](robot, puck);
	// printf("%i\n",robot->nxtSt);

}



// STATE FUNCTIONS
state standby(dd *robot, pk *puck){
	dd_disable(robot);
    // printf("standby\t");

    return ST_STANDBY;
}

state puck_search(dd *robot, pk *puck)
{
    //robot->veloDesired  = .1;
    robot->omegaDesired = .15; 
    // printf("puck_search\t");
    return ST_PK_SEARCH;
}


state puck_behind_persist(dd *robot, pk *puck)
{

    dd_goto_rot_trans(robot,.5);

    // printf("puck_behind\t");
    return ST_PK_BEHIND;
}
state puck_behind(dd *robot, pk *puck)
{
    robot->desLoc.x = robot->global.x;
	robot->desLoc.y = robot->global.y;
	robot->desLoc.th = ANG_REMAP(robot->global.th + PI);

    dd_goto_rot_trans(robot,.5);
    // printf("puck_behind\t");
    return ST_PK_BEHIND;
}


// state puck_pursue(dd *robot, pk *puck)
// {
// 	float kp = 2;
// 	float kd = 6;
// 	float k1 = .4;
// 	float k2 = 2;
// 	robot->omegaDesired = kp * puck->th + kd * (puck->th - puck->thPrev);
// 	robot->veloDesired = k1 / (k2 * ABS(robot->omegaDesired) + 1);
//     // printf("puck_pursue\t");
//     return ST_PK_PURSUE;
// }

state puck_pursue(dd *robot, pk *puck)
{
	float kp = 2;
	float kd = 6;
	float k1 = .4;
	float k2 = 2;
	float kap = 1.5;
	float kad = .1;
	static float prevAlpha = 0;
	float alpha = ANG_REMAP(robot->global.th + puck->th - PI/2);
	robot->omegaDesired = kp * puck->th  + kd * (puck->th - puck->thPrev)+ kap * alpha - kad * prevAlpha;
	robot->veloDesired = k1 / (k2 * ABS(robot->omegaDesired) + 1);
	prevAlpha = alpha;
    //printf("puck_pursue\t");
    return ST_PK_PURSUE;
}

state puck_to_goal(dd *robot, pk *puck)
{
    // printf("puck_to_goal\t");
	robot->desLoc.x = -62;
	robot->desLoc.y = -280;
	robot->desLoc.th = -PI/2;
	
	dd_goto_spiral(robot,.2);

    return ST_PK_TO_GOAL;
}

state goal_made(dd *robot, pk *puck)
{
	robot->desLoc.x = 0;
	robot->desLoc.y = 0;
	robot->desLoc.th = 0;
	
	dd_goto_rot_trans(robot,.5);
    // printf("goal_made\t");
    return ST_GOAL_MADE;
}

state fsm_error(dd *robot, pk *puck)
{
    // printf("fsm_error\t");
    return ST_PK_SEARCH;
}


// void main(void)
// {
// 	int i,j;
// 	state st;
// 	init_fsm();
// 	for(i=0;i<NUM_ST;i++){
// 		for(j=0;j<NUM_EV;j++){
// 			st = transArr[i][j]();
// 		}
// 		printf("\n");
// 	}
// }

