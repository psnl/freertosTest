/*
 * L1Timer.h
 *
 *  Created on: 3 dec. 2017
 *      Author: Patric
 */

#ifndef L1TIMER_H_
#define L1TIMER_H_

#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include <functional>

#define L1Time TickType_t

class L1Timer
{
public:

	void Start(const char* name, L1Time periodicTimeMs, std::function<void()> execute);
	static bool TimePassed(L1Time timePassed, L1Time timeSince);
	static void TimerCallback(TimerHandle_t pTimerHandle);
	static L1Time GetCurrentTime();
private:
	StaticTimer_t _timerBuffer;
	TimerHandle_t _timerHandle;
	std::function<void()> _execute;
};



#endif /* L1TIMER_H_ */
