#include "../framework/event.h"
#ifdef EVENT_H_
/******************************************************************************
* Includes
*******************************************************************************/
#include "mempool.h"

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
* Function Prototypes
*******************************************************************************/

/******************************************************************************
* Function Definitions
*******************************************************************************/

/* Return the address of allocated memory */
EvtId_t Event_New(eSignal sig, uint16_t evt_size)
{
	EvtId_t e = NULL;
	uint8_t idx;
	for(idx=0; idx < POOL_NUMB_SIZE; idx++)
	{
		if(evt_size <= (Mempool_P+idx)->Blocksize)
		{
			e = osMemoryPoolAlloc((Mempool_P+idx)->Handle, portWaitTimeout);
			if(e != (EvtId_t) NULL)
			{
				e->sig = sig;
				e->xdata.is_dynamic = (idx+1);
				e->xdata.ref_cnt = 0;
				break;
			}
		}
	}
	configASSERT(e); //FIXME: Cant alloc new event
	return e;
}

/* Event garbage collector */
void Event_GC(EvtId_t e)
{
	if (e->xdata.is_dynamic != 0)
	{
		portDISABLE_INTERRUPTS();
		/* The last one use this event ?*/
		if(e->xdata.ref_cnt == 0)
		{
			portENABLE_INTERRUPTS();
			uint8_t idx= e->xdata.is_dynamic - 1;
			osStatus_t status_free;
			status_free = osMemoryPoolFree((Mempool_P+idx)->Handle, e);
			configASSERT(status_free == osOK);
		}
		else
		{
			e->xdata.ref_cnt--;
			portENABLE_INTERRUPTS();
		}
	}
}
#endif /* EVENT_H_ */
