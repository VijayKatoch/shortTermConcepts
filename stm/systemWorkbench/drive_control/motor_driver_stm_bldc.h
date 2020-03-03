/*
 * motor_driver_stm_bldc.h
 *
 *  Created on: Feb 16, 2020
 *      Author: Vijay Katoch
 */

#ifndef MOTOR_DRIVER_STM_BLDC_H_
#define MOTOR_DRIVER_STM_BLDC_H_

#include "mbed.h"
#include "STSpin240_250.h"
#include "motor_driver.h"

#include<cstdint>

namespace KATBOT
{
  #define MAX_BDC_MOTORS 2

  class MotorDriverStmBLDC : public MotorDriver
  {
    public:

    MotorDriverStmBLDC();

    virtual void init();

    virtual void setDirection(std::uint32_t motorId, std::uint32_t direction);

    virtual void setSpeed(std::uint32_t motorId, std::uint32_t speed);

    virtual void run(std::uint32_t motorId);

    virtual void stop(uint32_t motorId, bool disableBridge = false);
    
    static void initStmBLDC(STSpin240_250 *motor);

    virtual ~MotorDriverStmBLDC();

    private:
      typedef struct MotorParam
      {
        std::uint32_t motorId;
        BDCMotor::direction_t direction;
      }MotorParam_t;

    private:
      STSpin240_250 *motor;
      STSpin240_250_init_t initVal;
      MotorParam_t motorParam[MAX_BDC_MOTORS];

  };
}


#endif /* MOTOR_DRIVER_STM_BLDC_H_ */
