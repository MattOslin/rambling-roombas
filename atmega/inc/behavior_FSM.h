//behavior_FSM.h

#ifndef BEHAVIOR_FSM_H_
#define BEHAVIOR_FSM_H_

#include "init.h"

struct state;

typedef void state_fn(struct state *);

struct state
{
    state_fn * next;
    int i; // data
};


void find_state( dd *robot, pk *puck);

#endif /* BEHAVIOR_FSM_H_ */