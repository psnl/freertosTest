/*
 * L2Config.h
 *
 *  Created on: 13 dec. 2017
 *      Author: Patric
 */

#ifndef L2CONFIG_H_
#define L2CONFIG_H_

#include "Interface/IHeater.h"

class L2Config : public IHeaterConfig
{
public:
	~L2Config(){}
	HeaterConfig* GetHeaterConfig();

};



#endif /* L2CONFIG_H_ */
