//behavior_FSM.c
//Controls the state of the system

#include <stdio.h>
#include "behavior_FSM.h"

//http://stackoverflow.com/questions/1647631/c-state-machine-design/1647679#1647679

//ONE OPTION
//Default state must be ZERO
typedef enum {ST_ERROR = -1, ST_PK_SEARCH, ST_PK_BEHIND, ST_PK_PURSUE, ST_PK_TO_GOAL, ST_GOAL_MADE, NUM_ST} state;
typedef enum {EV_ERROR = -1, EV_PK_OBSCURED, EV_PK_FOUND, EV_PK_BEHIND, EV_PK_OBTAINED, EV_GOAL_MADE, NUM_EV} event;

typedef state state_fn(dd *robot, pk *puck);

state_fn fsm_error, puck_search, puck_pursue, puck_to_goal, goal_made, puck_behind,puck_behind_persist;

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
	transArr[ST_PK_TO_GOAL]	[EV_PK_FOUND]		= &puck_pursue;
	transArr[ST_PK_TO_GOAL]	[EV_PK_FOUND]		= &puck_pursue;
	transArr[ST_PK_TO_GOAL]	[EV_GOAL_MADE]		= &goal_made;

	transArr[ST_GOAL_MADE]	[EV_PK_OBSCURED]	= &puck_search;

}

//BREAD AND BUTTER OF STATE MACHINE

void event_handler_fsm( dd *robot, pk *puck ){
	event ev;
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
		if (robot->ev == EV_PK_BEHIND && !dd_is_loc(robot,5)){
			ev = EV_PK_BEHIND;
		}
		else{
			ev = EV_PK_OBSCURED;
		}

	}
	robot->ev = ev;
	
}

void find_state( dd *robot, pk *puck)
{
	event_handler_fsm(robot,puck);
	printf("%i\t",robot->ev);
	robot->nxtSt = transArr[robot->nxtSt][robot->ev](robot, puck);
	printf("%i\n",robot->nxtSt);

}

// STATE FUNCTIONS
state puck_search(dd *robot, pk *puck)
{
    robot->veloDesired  = .1;
    robot->omegaDesired = .5; 
    printf("puck_search\t");
    return ST_PK_SEARCH;
}


state puck_behind_persist(dd *robot, pk *puck)
{

    dd_goto_rot_trans(robot,.5);

    printf("puck_behind\t");
    return ST_PK_BEHIND;
}
state puck_behind(dd *robot, pk *puck)
{
    robot->desLoc.x = robot->global.x;
	robot->desLoc.y = robot->global.y;
	robot->desLoc.th = ANG_REMAP(robot->global.th + PI);

    dd_goto_rot_trans(robot,.5);
    printf("puck_behind\t");
    return ST_PK_BEHIND;
}


state puck_pursue(dd *robot, pk *puck)
{
	int kp = 10;
	int kd = 0;
	int k = 1;
	robot->omegaDesired = - kp * puck->th - kd * (puck->th - puck->thPrev);
	robot->veloDesired = k/((abs(robot->omegaDesired) + 1));
    printf("puck_pursue\t");
    return ST_PK_PURSUE;
}

// state puck_pursue(dd *robot, pk *puck)
// {
// 	int kp = 10;
// 	int kd = 0;
// 	int k1 = 25;
// 	int k2 = 5;
// 	alpha = - robot->global.th - puck->th + pi/2;
// 	robot->omegaDesired = - kp * puck->th  - kd * (puck->th - puck->thPrev)+ k1 * alpha;
// 	robot->veloDesired = k2/((abs(robot->omegaDesired) + 1));
//     printf("puck_pursue\t");
//     return ST_PK_PURSUE;
// }

state puck_to_goal(dd *robot, pk *puck)
{
    printf("puck_to_goal\t");
    return ST_PK_TO_GOAL;
}

state goal_made(dd *robot, pk *puck)
{
    printf("goal_made\t");
    return ST_GOAL_MADE;
}

state fsm_error(dd *robot, pk *puck)
{
    printf("fsm_error\t");
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



// //ANOTHER OPTION
// int (* state[])(void) = { entry_state, foo_state, bar_state, exit_state};
// enum state_codes { entry, foo, bar, end};

// enum ret_codes { ok, fail, repeat};
// struct transition {
//     enum state_codes src_state;
//     enum ret_codes   ret_code;
//     enum state_codes dst_state;
// };
// /* transitions from end state aren't needed */
// struct transition state_transitions[] = {
//     {entry, ok,     foo},
//     {entry, fail,   end},
//     {foo,   ok,     bar},
//     {foo,   fail,   end},
//     {foo,   repeat, foo},
//     {bar,   ok,     end},
//     {bar,   fail,   end},
//     {bar,   repeat, foo}};

// #define EXIT_STATE end
// #define ENTRY_STATE entry

// int main(int argc, char *argv[]) {
//     enum state_codes cur_state = ENTRY_STATE;
//     enum ret_codes rc;
//     int (* state_fun)(void);

//     for (;;) {
//         state_fun = state[cur_state];
//         rc = state_fun();
//         if (EXIT_STATE == cur_state)
//             break;
//         cur_state = lookup_transitions(cur_state, rc);
//     }

//     return EXIT_SUCCESS;
// }