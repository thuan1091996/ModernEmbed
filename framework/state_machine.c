/******************************************************************************************************/
#include "../framework/state_machine.h"
/******************************************************************************
* State machine implementation 
*******************************************************************************/
static Evt Init_event = { .sig = INIT_SIG };
static Evt Entry_event = { .sig = ENTRY_SIG };
static Evt Exit_event = { .sig = EXIT_SIG };

/* StateMachine_Init 
 * Initialize for StateMachine instance */
void StateMachine_Init(StateMachine_t* const me, StateHandler initial_statehandler)
{
	me->statehandler = initial_statehandler;
	
	// Post Init signal
	(*me->statehandler)(me, &Init_event);	
}

void StateMachine_Dispatch(StateMachine_t* const me, EvtId_t const p_event)
{
	StateHandler prev_statehandler = me->statehandler; //Back up previous state-handler
	eStatus status = (*me->statehandler)(me, p_event);
	if(status == STATUS_TRANSITION)
	{
		(*prev_statehandler)(me, &Exit_event);
		(*me->statehandler)(me, &Entry_event);
	}
}
/******************************************************************************************************/
