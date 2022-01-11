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

Evt* Event_New(eSignal sig, uint16_t evt_size)
{
	Evt* e = NULL;
	uint8_t idx;
	for(idx=0; idx < POOL_NUMB_SIZE; idx++)
	{
		if(evt_size <= (Mempool_P+idx)->Blocksize)
		{
			e = osMemoryPoolAlloc((Mempool_P+idx)->Handle, portWaitTimeout);
			if(e != (Evt*) NULL)
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

#endif /* EVENT_H_ */
