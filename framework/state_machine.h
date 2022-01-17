#ifndef STATE_MACHINE_OBJ
#define STATE_MACHINE_OBJ
/******************************************************************************************************/
#include "stdint.h"
#include "stdbool.h"
#include "stdlib.h"
#include "string.h"
#include "event.h"

typedef enum  
{
	STATUS_INIT=0,     /* Pass to dispatch function when init */
	STATUS_IGNORE,     /* Not handle */
	STATUS_HANDLE,     /* Handle but not cause a transition */
	STATUS_TRANSITION  /* Handle and cause a transition */
}eStatus;

/* Forward declaration*/
typedef struct StateMachine StateMachine_t;
typedef eStatus (*StateHandler)(StateMachine_t* const me, EvtId_t const p_event);
/******************************************************************************
* State machine Object
*******************************************************************************/
#define TRANSITION(next_statehandler)  (me->statehandler = next_statehandler, STATUS_TRANSITION)

struct StateMachine
{
	StateHandler statehandler;
};
void StateMachine_Init(StateMachine_t* const me, StateHandler initial_statehandler);
void StateMachine_Dispatch(StateMachine_t* const me, EvtId_t const p_event);
/******************************************************************************************************/
#endif /* End of STATE_MACHINE_OBJ */
