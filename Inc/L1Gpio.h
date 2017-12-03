/*
 * L1Gpio.h
 *
 *  Created on: 3 dec. 2017
 *      Author: Patric
 */

#ifndef L1GPIO_H_
#define L1GPIO_H_

class L1Gpio
{
public:
	enum class Action
	{
		OFF,
		ON,
		TOGGLE
	};
static void Led(Action action);

};

#endif /* L1GPIO_H_ */
