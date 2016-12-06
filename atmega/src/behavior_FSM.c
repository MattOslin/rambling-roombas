//behavior_FSM.c
//Controls the state of the system

#include <stdio.h>
#include "behavior_FSM.h"

//http://stackoverflow.com/questions/1647631/c-state-machine-design/1647679#1647679

//ONE OPTION
//Default state must be ZERO
typedef enum {ST_ERROR = -1, ST_STANDBY, ST_PK_SEARCH, ST_PK_BEHIND, ST_PK_PURSUE, ST_PK_TO_GOAL, ST_GOAL_MADE, NUM_ST} state;
typedef enum {EV_ERROR = -1, EV_STANDBY, EV_PLAY, EV_PK_OBSCURED, EV_PK_FOUND, EV_PK_BEHIND, EV_PK_OBTAINED, EV_GOAL_MADE, NUM_EV} event;

typedef state state_fn(dd *rob, pk *puck);

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

	transArr[ST_STANDBY]    [EV_GOAL_MADE] 		= &goal_made;
    transArr[ST_STANDBY]    [EV_PLAY]			= &puck_search;
    transArr[ST_STANDBY]    [EV_PK_OBSCURED]    = &puck_search;
    transArr[ST_STANDBY]    [EV_PK_FOUND]		= &puck_pursue;
    transArr[ST_STANDBY]    [EV_PK_BEHIND] 		= &puck_behind;
    transArr[ST_STANDBY]    [EV_PK_OBTAINED]    = &puck_to_goal;
}

//BREAD AND BUTTER OF STATE MACHINE

void event_handler_fsm( dd *rob, pk *puck ){
	event ev = rob->ev;
	if(rob->enable){
		if (rob->goalMade){
			rob->goalMade = 0;
			rob->ev = EV_GOAL_MADE;
			return;
		}
		// if (rob->ev == EV_GOAL_MADE){
		// 	if(dd_is_loc(rob,5,.1)){
		// 		rob->ev = EV_STANDBY;
		// 		dd_disable(rob);
		// 	}
		// 	return;
		// }

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
			if (rob->ev == EV_PK_BEHIND && !dd_is_loc(rob,5,.1)){
				return;
			}
			else{
				ev = EV_PK_OBSCURED;
			}

		}
		rob->ev = ev;
	}
	else
	{
		rob->ev = EV_STANDBY;
	}
}


void find_state( dd *rob, pk *puck)
{
	event_handler_fsm(rob,puck);
	// printf("%i\t",rob->ev);
	rob->nxtSt = transArr[rob->nxtSt][rob->ev](rob, puck);
	// printf("%i\n",rob->nxtSt);

}



// STATE FUNCTIONS
state standby(dd *rob, pk *puck){
	dd_disable(rob);
    // printf("standby\t");

    return ST_STANDBY;
}

state puck_search(dd *rob, pk *puck)
{
    //rob->veloDesired  = .1;
    rob->omegaDesired = .15; 
    // printf("puck_search\t");
    return ST_PK_SEARCH;
}


state puck_behind_persist(dd *rob, pk *puck)
{

    dd_goto_rot_trans(rob,.5);

    // printf("puck_behind\t");
    return ST_PK_BEHIND;
}
state puck_behind(dd *rob, pk *puck)
{
    rob->desLoc.x = rob->global.x;
	rob->desLoc.y = rob->global.y;
	rob->desLoc.th = ANG_REMAP(rob->global.th + PI);

    dd_goto_rot_trans(rob,.5);
    // printf("puck_behind\t");
    return ST_PK_BEHIND;
}


// state puck_pursue(dd *rob, pk *puck)
// {
// 	float kp = 2;
// 	float kd = 6;
// 	float k1 = .4;
// 	float k2 = 2;
// 	rob->omegaDesired = kp * puck->th + kd * (puck->th - puck->thPrev);
// 	rob->veloDesired = k1 / (k2 * ABS(rob->omegaDesired) + 1);
//     // printf("puck_pursue\t");
//     return ST_PK_PURSUE;
// }

state puck_pursue(dd *rob, pk *puck)
{
	float kp = 2;
	float kd = 6;
	float k1 = .4;
	float k2 = 2;
	float kap = 0;//.6;
	float kad = 0;//.1;
	static float prevAlpha = 0;
	float alpha = ANG_REMAP(rob->global.th + puck->th - rob->direction * PI/2);
	rob->omegaDesired = kp * puck->th  + kd * (puck->th - puck->thPrev)+ kap * alpha - CTRL_FREQ * kad * (alpha - prevAlpha);
	rob->veloDesired = k1 / (k2 * ABS(rob->omegaDesired) + 1);
	// dd_norm(rob,.4);
	prevAlpha = alpha;

    //printf("puck_pursue\t");
    return ST_PK_PURSUE;
}

state puck_to_goal(dd *rob, pk *puck)
{
	rob->desLoc.x = 0;
	rob->desLoc.y = rob->direction * 300;
	rob->desLoc.th = rob->direction * PI/2;
	
	dd_goto(rob, puck, .4);
	shoot_puck(rob, puck);
    // static float prevAlpha = 0;
    // static float prevPhi = 0;
    // float kp = 2;
    // float kd = 6;
    // float k1 = .4;
    // float k2 = 2;
    // float kap = 0;//1.5;
    // float kad = 0;//;.1;
    // float gamma = atan2(rob->desLoc.y - rob->global.y, rob->desLoc.x - rob->global.x);
    // float phi = ANG_REMAP(gamma - rob->global.th);
    // float alpha = ANG_REMAP(gamma - rob->desLoc.th);
    // rob->omegaDesired = kp * phi  + kd * (phi - prevPhi)+ kap * alpha - kad * prevAlpha;
    // rob->veloDesired = k1 / (k2 * ABS(rob->omegaDesired) + 1);
    // if (rob->isHave){
    // 	rob->veloDesired = MAX(rob->veloDesired, 2 * MIN_PUCK_TURN_RAD * ABS(rob->omegaDesired) / WHEEL_RADIAL_LOC);
    // }
    // dd_norm(rob,.4);
    // prevAlpha = alpha;
    // prevPhi = phi;   

    // DEBUG
 	// m_usb_tx_string(" alpha: ");
	// m_usb_tx_int(100*alpha);
	// m_usb_tx_string(" phi: ");
	// m_usb_tx_int(100*phi);	
	// m_usb_tx_string(" gamma: ");
	// m_usb_tx_int(100*gamma);

    // printf("puck_to_goal\t");
    return ST_PK_TO_GOAL;

}

state goal_made(dd *rob, pk *puck)
{
	rob->desLoc.x = 0;
	rob->desLoc.y = 0;
	rob->desLoc.th = 0;
	
    // printf("goal_made\t");
    return ST_STANDBY;
}

state fsm_error(dd *rob, pk *puck)
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

