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
#ifndef FREE_ACT_H
#define FREE_ACT_H

/******************************************************************************
* Includes
*******************************************************************************/
#include "cmsis_os.h"
#include "queue.h"
#include "state_machine.h"
#include "mempool.h"

/******************************************************************************
* Preprocessor Constants
*******************************************************************************/

/******************************************************************************
* Configuration Constants
*******************************************************************************/
/* CMSIS RTOS v2 */
#define	portTHREAD_ATTR_T			osThreadAttr_t
#define	portTHREAD_HANDLE_T			osThreadId_t

#define	portEQUEUE_ATTR_T			osMessageQueueAttr_t
#define	portEQUEUE_HANDLE_T			osMessageQueueId_t

/******************************************************************************
* Macros
*******************************************************************************/


/******************************************************************************
* Typedefs
*******************************************************************************/
typedef struct Active Active; /* forward declaration */

/* Active Object base class */
struct Active {

	StateMachine_t				sm;		 			/* state machine*/

	portTHREAD_HANDLE_T			thread_handle;		/* private thread */
	portTHREAD_ATTR_T const*	thread_param;

	/* Multiple-write / Single read access */
	portEQUEUE_HANDLE_T			equeue_handle;		/* private message queue */
	portEQUEUE_ATTR_T const*	equeue_param;
	uint32_t					equeue_len;

    /* active object data added in subclasses of Active */
};

/******************************************************************************
* Variables
*******************************************************************************/


/******************************************************************************
* Function Prototypes
*******************************************************************************/
void Active_Init(Active *const				me,
				 StateHandler				initial_statehandler,
				 portTHREAD_ATTR_T const*	p_thread_attr,
				 portEQUEUE_ATTR_T const*	p_equeue_attr,
				 uint32_t					equeue_max_len);

void Active_post(Active * const me, Evt const * const e);

#if 0
void Active_start(Active * const me,
                  uint8_t prio,       /* priority (1-based) */
                  Evt **queueSto,
                  uint32_t queueLen,
                  void *stackSto,
                  uint32_t stackSize,
                  uint16_t opt);

void Active_postFromISR(Active * const me, Evt const * const e,
                        BaseType_t *pxHigherPriorityTaskWoken);

/*---------------------------------------------------------------------------*/
/* Time Event facilities... */

/* Time Event class */
typedef struct {
    Evt super;       	/* inherit Evt */
    Active *act;       /* the AO that requested this TimeEvent */
    uint32_t timeout;  /* timeout counter; 0 means not armed */
    uint32_t interval; /* interval for periodic TimeEvent, 0 means one-shot */
} TimeEvent;

void TimeEvent_ctor(TimeEvent * const me, eSignal sig, Active *act);
void TimeEvent_arm(TimeEvent * const me, uint32_t timeout, uint32_t interval);
void TimeEvent_disarm(TimeEvent * const me);

/* static (i.e., class-wide) operation */
void TimeEvent_tickFromISR(BaseType_t *pxHigherPriorityTaskWoken);
#endif  /* End of 0 */

#endif /* FREE_ACT_H */
