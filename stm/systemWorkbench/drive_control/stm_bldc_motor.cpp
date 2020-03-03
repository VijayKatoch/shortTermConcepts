/*
 * stm_bldc_motor.cpp
 *
 *  Created on: Mar 3, 2020
 *      Author: Vijay Katoch
 */

#include "stm_bldc_motor.h"
#include "motor_driver_stm_bldc.h"


using namespace KATBOT;

StmBLDCMotor::StmBLDCMotor()
{
  motorDriver = new MotorDriverStmBLDC();
}


