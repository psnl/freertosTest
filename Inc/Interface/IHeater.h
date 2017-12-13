/*
 * IHeater.h
 *
 *  Created on: 13 dec. 2017
 *      Author: Patric
 */

#ifndef INTERFACE_IHEATER_H_
#define INTERFACE_IHEATER_H_

class IHeaterStatus
{
public:
	enum class HeaterAlert
	{
		TEMP_HIGH_ALARM = 0,
		TEMP_LOW_ALARM = 1,
		HEATER_ON_TIME = 2,
		HEATER_FAILED = 3
	};
	virtual ~IHeaterStatus(){}

	virtual void Alert(HeaterAlert alert)=0;
	virtual bool HeaterOk()=0;
	virtual void HeaterOn()=0;
	virtual void HeaterOff()=0;
	virtual int Temperature()=0;
};


class IHeaterConfig
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
	};

	virtual ~IHeaterConfig(){}
	virtual HeaterConfig* GetHeaterConfig() = 0;
};



#endif /* INTERFACE_IHEATER_H_ */
