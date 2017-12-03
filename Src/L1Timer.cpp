/*
 * L1Timer.cpp
 *
 *  Created on: 3 dec. 2017
 *      Author: Patric
 */

#include <L1Timer.h>
static L1Timer* pL1Timer;

void L1Timer::Start(const char* name, L1Time ulPeriodicTimeMs, std::function<void()> execute)
{
	osTimerDef(name, TimerCallback);
	_execute = execute;
	pL1Timer = this;
	_timerHandle = osTimerCreate(osTimer(name), osTimerPeriodic, pL1Timer);

	osTimerStart(_timerHandle, ulPeriodicTimeMs);
}

void L1Timer::TimerCallback(void const * argument)
{

	//HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    //((L1Timer*)argument)->_execute();
    pL1Timer->_execute();
}



bool L1Timer::TimePassed(L1Time timePassed, L1Time timeSince)
{
#if( configUSE_16_BIT_TICKS == 1 )
	#error ("Only uin32_t support for timer")
#else
	uint32_t u  = ((L1Timer::GetCurrentTime()+timePassed) - timeSince);
	return (u & 0x8000000u);
#endif
}

L1Time L1Timer::GetCurrentTime()
{
	return (L1Time)xTaskGetTickCount();
}
