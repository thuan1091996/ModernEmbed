/*
 * Event.h
 *
 *  Created on: Jan 5, 2022
 *      Author: thuantm
 */

#ifndef EVENT_H_
#define EVENT_H_
/******************************************************************************
* Event
*******************************************************************************/
/* Forward declaration*/
typedef uint16_t eSignal;
typedef struct Event_t Evt;

typedef enum {
	INIT_SIG=1,
	ENTRY_SIG,
	EXIT_SIG,
	USER_SIG,
}ReservedSignals;

enum Signal
{
	/* BLE signals */
	DEFAULT_SIG = USER_SIG,
	/* .... */
	SIG_MAX
};

struct Event_t
{
	eSignal sig;						//Inheritance

};

#endif /* EVENT_H_ */
