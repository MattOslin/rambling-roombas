//behavior_FSM.c
//Controls the state of the system
#include "behavior_FSM.h"
#include <stdio.h>
//http://stackoverflow.com/questions/1647631/c-state-machine-design/1647679#1647679

//ONE OPTION



typedef enum {ST_ERROR = -1, ST_PK_SEARCH, ST_PK_PURSUE, ST_PK_TO_GOAL, ST_GOAL_MADE, NUM_ST} state;

typedef enum {EV_ERROR = -1, EV_PK_FOUND, EV_PK_OBSCURED, EV_PK_OBTAINED, EV_PK_LOST, EV_GOAL_MADE, NUM_EV} event;

typedef state state_fn(void);

state_fn fsm_error, puck_search, puck_pursue, puck_to_goal, goal_made;

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
	transArr[ST_PK_PURSUE]	[EV_PK_FOUND]		= &puck_pursue;
	transArr[ST_PK_PURSUE]	[EV_PK_OBSCURED]	= &puck_search;
	transArr[ST_PK_PURSUE]	[EV_PK_OBTAINED]	= &puck_to_goal;
	transArr[ST_PK_TO_GOAL]	[EV_PK_OBTAINED]	= &puck_to_goal;
	transArr[ST_PK_TO_GOAL]	[EV_PK_LOST]		= &puck_pursue;
	transArr[ST_PK_TO_GOAL]	[EV_GOAL_MADE]		= &goal_made;
	transArr[ST_GOAL_MADE]	[EV_PK_OBSCURED]	= &puck_search;
}

event event_handler_fsm(dd *robot,pk *puck){
	static int i = -1;
	i++;
	return i%NUM_EV;
}

void find_state(dd *robot,pk *puck)
{
	static state st = ST_PK_SEARCH;
	event ev = event_handler_fsm(robot,puck);
	printf("%i\t",ev);
	st = transArr[st][ev]();

}



state puck_search(void)
{
    printf("puck_search\t");
    return ST_PK_SEARCH;
}

state puck_pursue(void)
{
    printf("puck_pursue\t");
    return ST_PK_PURSUE;
}

state puck_to_goal(void)
{
    printf("puck_to_goal\t");
    return ST_PK_TO_GOAL;
}

state goal_made(void)
{
    printf("goal_made\t");
    return ST_GOAL_MADE;
}

state fsm_error(void)
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