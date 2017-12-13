/*
 * L1Timer.cpp
 *
 *  Created on: 3 dec. 2017
 *      Author: Patric
 */

#include <L1Timer.h>
static TimerHandle_t timerHandle;

void L1Timer::Start(const char* name, L1Time ulPeriodicTimeMs, std::function<void()> execute)
{
	_execute = execute;
	//pL1Timer = this;
	timerHandle = xTimerCreateStatic(name, 100, pdTRUE, this,  TimerCallback, &_timerBuffer);
	//vTimerSetTimerID(timerHandle, this);

	xTimerStart(timerHandle, 0);
}

void L1Timer::TimerCallback(TimerHandle_t pTimerHandle)
{
	//L1Timer* pL1Timer = (L1Timer*)pvTimerGetTimerID(pTimerHandle);
	HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    //((L1Timer*)argument)->_execute();
    //pL1Timer->_execute();

	uint32_t ulRef = 2048;
	uint32_t ulMeasured = 1651;
	uint32_t ulMax = 4095;

	uint32_t ulValue = (ulMeasured * ulRef) / ulMax;

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
