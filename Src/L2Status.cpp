/*
 * Status.cpp
 *
 *  Created on: 2 dec. 2017
 *      Author: Patric
 */
#include <L2Status.h>
#include "L1Gpio.h"

static L2Status* pL2Status;

void L2Status::Start()
{
	state = State::NOT_INITED;
	stateTime = L1Timer::GetCurrentTime();
	pL2Status = this;
	l1Timer.Start("L2Status", 100, [pL2Status]() {pL2Status->Update();});
	//l1Timer.Start("L2Status", 100, [this]() {L1Gpio::Led(L1Gpio::Action::TOGGLE);});
	//l1Timer.Start("L2Status", 100, nullptr);
}

void L2Status::Update()
{
	switch (state)
	{
	case State::NOT_INITED:
		L1Gpio::Led(L1Gpio::Action::OFF);
		state = State::LED_OFF;
		stateTime = L1Timer::GetCurrentTime();
		break;
	case State::LED_OFF:
		if (L1Timer::TimePassed(900, stateTime))
		{
			L1Gpio::Led(L1Gpio::Action::ON);
			state = State::LED_ON;
			stateTime = L1Timer::GetCurrentTime();
		}
		break;
	case State::LED_ON:
		if (L1Timer::TimePassed(100, stateTime))
		{
			L1Gpio::Led(L1Gpio::Action::OFF);
			state = State::LED_OFF;
			stateTime = L1Timer::GetCurrentTime();
		}
		break;
	}
}



