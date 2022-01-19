/******************************************************************************
* Includes
*******************************************************************************/
#include "../framework/ps.h"

/******************************************************************************
* Module Preprocessor Constants
*******************************************************************************/

/******************************************************************************
* Module Preprocessor Macros
*******************************************************************************/

/******************************************************************************
* Module Typedefs
*******************************************************************************/
typedef  struct {
	uint32_t bits;
}Subscribers_t ;


/******************************************************************************
* Module Variable Definitions
*******************************************************************************/
static Subscribers_t Subscribers_List[SIG_MAX]={0};

/******************************************************************************
* Function Prototypes
*******************************************************************************/

/******************************************************************************
* Function Definitions
*******************************************************************************/
void Subscribe(ActiveId_t const actor, EvtId_t e)
{
	uint8_t idx = Active_GetID(actor);
	configASSERT(idx < ACTOR_MAX_NUMB)

	// Set index bit in subscribe list
	Subscribers_List[e->sig].bits |= (1 << idx);
}

void UnSubscribe(ActiveId_t const actor, EvtId_t e)
{
	uint8_t idx = Active_GetID(actor);
	configASSERT(idx < ACTOR_MAX_NUMB)

	// Reset index bit in subscribe list
	Subscribers_List[e->sig].bits &= ~(1 << idx);
}

void UnSubscribeAll(ActiveId_t const actor)
{
	uint8_t actor_id =  Active_GetID(actor);
	for(uint8_t idx; idx < SIG_MAX; idx++)
	{
		Subscribers_List[idx].bits &= ~(1 << actor_id);
	}
}

void Publish(EvtId_t e)
{
	portDISABLE_INTERRUPTS();

	for(uint8_t idx=0; idx< ACTOR_MAX_NUMB; idx++)
	{
		if((Subscribers_List[e->sig].bits & (1 << idx)) != 0)
		{
			ActiveId_t dest_active = Active_GetActiveByID(idx);

			configASSERT(dest_active);

			//Post message
			Active_post(dest_active , e);
		}
	}
	portENABLE_INTERRUPTS();
}
