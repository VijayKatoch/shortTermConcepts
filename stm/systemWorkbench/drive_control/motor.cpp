/*
 * motor.cpp
 *
 *  Created on: Mar 3, 2020
 *      Author: Vijay Katoch
 */

#include "motor.h"
#include <assert.h>

using namespace KATBOT;
using namespace std;

void Motor::setMotorDriver(MotorDriver* md)
{
  assert(md!=NULL);
  motorDriver = md;

}

void Motor::init()
{
  assert(motorDriver!=NULL);
  motorDriver->init();
}

void Motor::setDirection(uint32_t motorId, uint32_t direction)
{
  assert(motorDriver!=NULL);
  motorDriver->setDirection(motorId, direction);
}

void Motor::setSpeed(uint32_t motorId, uint32_t speed)
{
  assert(motorDriver!=NULL);
  motorDriver->setSpeed(motorId, speed);
}


void Motor::run(uint32_t motorId)
{
  assert(motorDriver!=NULL);
  motorDriver->run(motorId);
}

void Motor::stop(uint32_t motorId, bool disableBridge)
{
  assert(motorDriver!=NULL);
  motorDriver->stop(motorId, disableBridge);
}
