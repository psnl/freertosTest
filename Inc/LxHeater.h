/*
 * LxHeater.h
 *
 *  Created on: 9 dec. 2017
 *      Author: Patric
 */

#ifndef LXHEATER_H_
#define LXHEATER_H_

#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include "Interface/IHeater.h"

class IConfig
{
public:
	struct HeaterConfig
	{
		int heaterLow;
		int heaterLowAlert;
		int heaterHigh;
		int heaterHighAlert;
		unsigned int heaterMaxOnTimeSec;
		unsigned int heaterHysteresisSec;

		HeaterConfig()
		{
			heaterLow = 10;
			heaterLowAlert = 5;
			heaterHigh = 15;
			heaterHighAlert = 60;
			heaterMaxOnTimeSec = 3600;
			heaterHysteresisSec = 10;
		}
	};

	HeaterConfig* GetHeaterConfig();
};



class LxHeater
{
public:
	void Start(IHeaterConfig* config, IHeaterStatus* status);

	void Task();
private:
	IHeaterConfig* _config;
	IHeaterStatus* _status;
	bool _heaterOn;
	unsigned int _heaterOnTime;
	unsigned int _heaterHysteresisCount;
};



#endif /* LXHEATER_H_ */
