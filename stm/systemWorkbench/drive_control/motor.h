/*
 * motor.h
 *
 *  Created on: Mar 3, 2020
 *      Author: Vijay Katoch
 */

#ifndef MOTOR_H_
#define MOTOR_H_

#include<cstdint>
#include <cstddef>

#include "motor_driver.h"

namespace KATBOT
{
  class Motor
  {
    public:

      MotorDriver *motorDriver;

      Motor() : motorDriver(NULL)
      {}

      void setMotorDriver(MotorDriver* md);

      /* Motor driver interface*/
      void init();
      void setDirection(std::uint32_t motorId, std::uint32_t direction);
      void setSpeed(std::uint32_t motorId, std::uint32_t speed);
      void run(std::uint32_t motorId);
      void stop(std::uint32_t motorId, bool disableBridge = false);
  };
}

#endif /* MOTOR_H_ */
