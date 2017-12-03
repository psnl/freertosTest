/*
 * L1Gpio.cpp
 *
 *  Created on: 3 dec. 2017
 *      Author: Patric
 */
#include "L1Gpio.h"
#include "stm32f1xx_hal.h"

void L1Gpio::Led(Action action)
{
	switch(action)
	{
	case Action::OFF:
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
		break;
	case Action::ON:
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
		break;
	case Action::TOGGLE:
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		break;
	}
}

