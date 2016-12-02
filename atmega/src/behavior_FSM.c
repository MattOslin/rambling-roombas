//behavior_FSM.c
//Controls the state of the system




//ONEOPTION
#include <stdio.h>

struct state;
typedef void state_fn(struct state *);

struct state
{
    state_fn * next;
    int i; // data
};

state_fn foo, bar;

void foo(struct state * state)
{
    printf("%s %i\n", __func__, ++state->i);
    state->next = bar;
}

void bar(struct state * state)
{
    printf("%s %i\n", __func__, ++state->i);
    state->next = state->i < 10 ? foo : 0;
}

// int main(void)
// {
//     struct state state = { foo, 0 };
//     while(state.next) state.next(&state);
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