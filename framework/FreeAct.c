/*****************************************************************************
* Free Active Object pattern implementation (FreeAct) based on FreeRTOS
*
*                    Q u a n t u m  L e a P s
*                    ------------------------
*                    Modern Embedded Software
*
* Copyright (C) 2020 Quantum Leaps, LLC. All rights reserved.
*
* MIT License:
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to
* deal in the Software without restriction, including without limitation the
* rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
* sell copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*
* Contact information:
* <www.state-machine.com>
* <info@state-machine.com>
*****************************************************************************/


/******************************************************************************
* Includes
*******************************************************************************/
#include "FreeAct.h" /* Free Active Object interface */

/******************************************************************************
* Module Preprocessor Constants
*******************************************************************************/

/******************************************************************************
* Module Preprocessor Macros
*******************************************************************************/

/******************************************************************************
* Module Typedefs
*******************************************************************************/

/******************************************************************************
* Module Variable Definitions
*******************************************************************************/
static ActiveId_t Active_List[ACTOR_MAX_NUMB]={0};


/******************************************************************************
* Private Functions
*******************************************************************************/
/*..........................................................................*/
/* Event-loop thread function for all Active Objects (FreeRTOS task signature) */
static void Active_eventLoop(void *pvParameters) {
	Active *me = (Active *)pvParameters;
	configASSERT(me); /* Active object must be provided */

	for (;;) {   /* for-ever "superloop" */

		EvtId_t e; /* pointer to event object ("message") */

		/* wait for any event and receive it into object 'e' */

		osMessageQueueGet(me->equeue_handle, (void*)&e, NULL ,portWaitTimeout); /* BLOCKING! */

		configASSERT(e);

		/* dispatch event to the active object 'me' */
		StateMachine_Dispatch(&me->sm, e);			/* NO BLOCKING! */

		Event_GC(e);

	}
}


/******************************************************************************
* Function Definitions
*******************************************************************************/
void Active_Init(Active *const				me,
				 StateHandler				initial_statehandler,
				 portTHREAD_ATTR_T const*	p_thread_attr,
				 portEQUEUE_ATTR_T const*	p_equeue_attr,
				 uint32_t					equeue_max_len)
{
	static uint8_t active_id = 0;
    configASSERT(me); /* Active object must be provided */
	StateMachine_Init(&me->sm, initial_statehandler);

	/* Initialize the Thread */
	osThreadId_t thread_status = osThreadNew(&Active_eventLoop, me, p_thread_attr);
    configASSERT(thread_status);
	me->thread_handle = thread_status;
	me->thread_param = p_thread_attr;

	/* Initialize the Event queue */
	osMessageQueueId_t equeue_status;
	equeue_status = osMessageQueueNew(equeue_max_len, sizeof(EvtId_t), p_equeue_attr);
    configASSERT(equeue_status);
	me->equeue_handle = equeue_status;
	me->equeue_param = p_equeue_attr;
	Active_List[active_id++] = me;
}

uint8_t Active_GetID(Active* const me)
{
	for(uint8_t idx=0; idx < ACTOR_MAX_NUMB; idx++)
	{
		if(me == Active_List[idx])
			return idx;
	}
	return 0xFF;
}

ActiveId_t Active_GetActiveByID(uint8_t id)
{
	if(id <= ACTOR_MAX_NUMB)
	{
		return Active_List[id];
	}
	return NULL;
}

/*..........................................................................*/
bool Active_post(Active * const me, EvtId_t const e){

	bool ret = false;
	osStatus_t status;
	for(uint8_t count=0; count < ACTOR_MAX_RETRY; count++)
	{
		status = osMessageQueuePut(me->equeue_handle, &e, 0, 0);
		if (status == osOK)
		{
			if(e->xdata.is_dynamic != 0)
			{
				portDISABLE_INTERRUPTS();
				e->xdata.ref_cnt++;
				portENABLE_INTERRUPTS();
			}
			ret = true;
			break;
		}
		else if (status == osErrorTimeout)
		{
			osDelay(50); /* Retry to put in case of full after 50 ticks */
		}
		else
		{
			configASSERT(0);
		}
	}
	return ret;
}

