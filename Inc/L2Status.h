/*
 * Status.h
 *
 *  Created on: 2 dec. 2017
 *      Author: Patric
 */

#ifndef L2STATUS_H_
#define L2STATUS_H_

#include "L1Timer.h"

class L2Status
{
public:
	enum class State
	{
		NOT_INITED = 0,
		LED_ON = 1,
		LED_OFF = 2
	};

	void Start();
private:

	void Update();

	L1Timer l1Timer;
	State state;
	L1Time stateTime;
};



#endif /* L2STATUS_H_ */
