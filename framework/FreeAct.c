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
}

/*..........................................................................*/
bool Active_post(Active * const me, EvtId_t const e){

	bool ret = false;
	if ( osMessageQueueGetSpace(me->equeue_handle) > 0)
	{
		osStatus_t status = osMessageQueuePut(me->equeue_handle, &e, 0, portWaitTimeout);
		if (status != osOK) {

			configASSERT(0);
		}
		else
		{
			if(e->xdata.is_dynamic != 0)
			{
				portDISABLE_INTERRUPTS();
				e->xdata.ref_cnt++;
				portENABLE_INTERRUPTS();
			}
			ret = true;
		}
	}
	return ret;
}

#if 0
/*..........................................................................*/
void Active_postFromISR(Active * const me, Evt const * const e,
                        BaseType_t *pxHigherPriorityTaskWoken)
{
    BaseType_t status = xQueueSendFromISR(me->queue, (void *)&e,
                                          pxHigherPriorityTaskWoken);
    configASSERT(status == pdTRUE);
}

/*--------------------------------------------------------------------------*/
/* Time Event services... */

static TimeEvent *l_tevt[10]; /* all TimeEvents in the application */
static uint_fast8_t l_tevtNum; /* current number of TimeEvents */

/*..........................................................................*/
void TimeEvent_ctor(TimeEvent * const me, eSignal sig, Active *act) {
    /* no critical section because it is presumed that all TimeEvents
    * are created *before* multitasking has started.
    */
    me->super.sig = sig;
    me->act = act;
    me->timeout = 0U;
    me->interval = 0U;

    /* register one more TimeEvent with the application */
    configASSERT(l_tevtNum < sizeof(l_tevt)/sizeof(l_tevt[0]));
    l_tevt[l_tevtNum] = me;
    ++l_tevtNum;
}

/*..........................................................................*/
void TimeEvent_arm(TimeEvent * const me, uint32_t timeout, uint32_t interval) {
    taskENTER_CRITICAL();
    me->timeout = timeout;
    me->interval = interval;
    taskEXIT_CRITICAL();
}

/*..........................................................................*/
void TimeEvent_disarm(TimeEvent * const me) {
    taskENTER_CRITICAL();
    me->timeout = 0U;
    taskEXIT_CRITICAL();
}

/*..........................................................................*/
void TimeEvent_tickFromISR(BaseType_t *pxHigherPriorityTaskWoken) {
    uint_fast8_t i;
    for (i = 0U; i < l_tevtNum; ++i) {
        TimeEvent * const t = l_tevt[i];
        configASSERT(t); /* TimeEvent instance must be registered */
        if (t->timeout > 0U) { /* is this TimeEvent armed? */
            if (--t->timeout == 0U) { /* is it expiring now? */
                Active_postFromISR(t->act, &t->super,
                		pxHigherPriorityTaskWoken);
                t->timeout = t->interval; /* rearm or disarm (one-shot) */
           }
        }
    }
}
#endif  /* End of 0 */
