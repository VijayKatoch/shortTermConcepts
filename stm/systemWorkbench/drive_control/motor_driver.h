/*
 * motor_driver.h
 *
 *  Created on: Feb 16, 2020
 *      Author: Vijay Katoch
 */

#ifndef MOTOR_DRIVER_H_
#define MOTOR_DRIVER_H_

#include<cstdint>

namespace KATBOT
{
  class MotorDriver
  {
    public:

      MotorDriver();

      virtual void init() = 0;

      virtual void setDirection(std::uint32_t motorId,
          std::uint32_t direction) = 0;

      virtual void setSpeed(std::uint32_t motorId, std::uint32_t speed) = 0;

      virtual void run(std::uint32_t motorId) = 0;

      virtual void stop(std::uint32_t motorId, bool disableBridge = false) = 0;

      virtual ~MotorDriver();

  };
}



#endif /* MOTOR_DRIVER_H_ */
