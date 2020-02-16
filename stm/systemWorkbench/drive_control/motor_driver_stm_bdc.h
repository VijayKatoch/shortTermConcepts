/*
 * motor_driver_stm_bdc.h
 *
 *  Created on: Feb 16, 2020
 *      Author: Vijay Katoch
 */

#ifndef MOTOR_DRIVER_STM_BDC_H_
#define MOTOR_DRIVER_STM_BDC_H_

#include "mbed.h"
#include "STSpin240_250.h"
#include "motor_driver.h"

namespace KATBOT
{
  #define MAX_BDC_MOTORS 2

  class MotorDriverStmBDC : public MotorDriver
  {
    public:

    MotorDriverStmBDC();

    void init();

    void setDirection(std::uint32_t motorId, std::uint32_t direction);

    void setSpeed(std::uint32_t motorId, std::uint32_t speed);

    void run(std::uint32_t motorId);

    void stop(uint32_t motorId, bool disableBridge = false);

    virtual ~MotorDriverStmBDC();

    private:
      typedef struct MotorParam
      {
        uint32_t motorId;
        BDCMotor::direction_t direction;
      }MotorParam_t;

    private:
      STSpin240_250 *motor;
      STSpin240_250_init_t initVal;
      MotorParam_t motorParam[MAX_BDC_MOTORS];

  };
}


#endif /* MOTOR_DRIVER_STM_BDC_H_ */
