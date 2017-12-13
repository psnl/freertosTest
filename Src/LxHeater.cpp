/*
 * LxHeater.cpp
 *
 *  Created on: 9 dec. 2017
 *      Author: Patric
 */

#include "LxHeater.h"

void LxHeater::Start(IConfig* config, IStatus* status)
{
	_config = config;
	_status = status;
	_heaterOn = false;
	_heaterOnTime = 0;
	_heaterHysteresisCount = _config->GetHeaterConfig()->heaterHysteresisSec;
}

void LxHeater::Task()
{
	//Todo: Get all variable one in task
    while (true) {
		if (!_status->HeaterOk()) {
			_status->Alert(IStatus::HeaterAlert::HEATER_FAILED);
		}
		if (_heaterOn && (_heaterOnTime < _config->GetHeaterConfig()->heaterMaxOnTimeSec)) {
			_heaterOnTime++;
		}
		if (_heaterOnTime >= _config->GetHeaterConfig()->heaterMaxOnTimeSec) {
			_status->Alert(IStatus::HeaterAlert::HEATER_ON_TIME);
		}

		_heaterHysteresisCount++;
		if (_heaterHysteresisCount >= _config->GetHeaterConfig()->heaterHysteresisSec) {
			_heaterHysteresisCount = 0;

			if (_status->Temperature() < _config->GetHeaterConfig()->heaterLow) {
				_status->HeaterOn();
				if (_status->Temperature() < _config->GetHeaterConfig()->heaterLowAlert) {
					_status->Alert(IStatus::HeaterAlert::TEMP_LOW_ALARM);
				}
			}
			else if (_status->Temperature() > _config->GetHeaterConfig()->heaterHigh) {
				_status->HeaterOff();
				if (_status->Temperature() > _config->GetHeaterConfig()->heaterHighAlert) {
					_status->Alert(IStatus::HeaterAlert::TEMP_HIGH_ALARM);
				}
			}
		}
		vTaskDelay(1000);
    }
}




