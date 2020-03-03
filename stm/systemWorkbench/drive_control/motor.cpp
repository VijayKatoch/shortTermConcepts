/*
 * motor.cpp
 *
 *  Created on: Mar 3, 2020
 *      Author: Vijay Katoch
 */

#include "motor.h"
#include <assert.h>

using namespace KATBOT;

void Motor::setMotorDriver(MotorDriver* md)
{
  assert(md!=nullptr);
  motorDriver = md;

}

void Motor::init()
{
  assert(motorDriver!=nullptr);
  motorDriver->init();
}

void Motor::setDirection(uint32_t motorId, uint32_t direction)
{
  assert(motorDriver!=nullptr);
  motorDriver->setDirection(motorId, direction);
}

void Motor::setSpeed(uint32_t motorId, uint32_t speed)
{
  assert(motorDriver!=nullptr);
  motorDriver->setSpeed(motorId, speed);
}


void Motor::run(uint32_t motorId)
{
  assert(motorDriver!=nullptr);
  motorDriver->run(motorId);
}

void Motor::stop(uint32_t motorId, bool disableBridge = false)
{
  assert(motorDriver!=nullptr);
  motorDriver->stop(motorId, disableBridge);
}






