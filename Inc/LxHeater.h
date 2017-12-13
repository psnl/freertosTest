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

class IStatus
{
public:
	enum class HeaterAlert
	{
		TEMP_HIGH_ALARM = 0,
		TEMP_LOW_ALARM = 1,
		HEATER_ON_TIME = 2,
		HEATER_FAILED = 3
	};

	void Alert(HeaterAlert alert);
	bool HeaterOk();
	void HeaterOn();
	void HeaterOff();
	int Temperature();
};

class LxHeater
{
public:
	void Start(IConfig* config, IStatus* status);

	void Task();
private:
	IConfig* _config;
	IStatus* _status;
	bool _heaterOn;
	unsigned int _heaterOnTime;
	unsigned int _heaterHysteresisCount;
};



#endif /* LXHEATER_H_ */
