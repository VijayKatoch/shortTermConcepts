/*
 * motor_driver_stm_bldc.cpp
 *
 *  Created on: Feb 16, 2020
 *      Author: Vijay Katoch
 */

#include "motor_driver_stm_bldc.h"

using namespace KATBOT;
using namespace std;

MotorDriverStmBLDC::MotorDriverStmBLDC() : motor(NULL)
{
  /* Frequency of PWM of Input Bridge A in Hz up to 100000Hz */
  initVal.bridgePwmFreq[0] = 20000;
  /* Frequency of PWM of Input Bridge B in Hz up to 100000Hz */
  initVal.bridgePwmFreq[1] = 20000;
  /* Frequency of PWM used for Ref pin in Hz up to 100000Hz  */
  initVal.refPwmFreq = 20000;
  /* Duty cycle of PWM used for Ref pin (from 0 to 100)      */
  initVal.refPwmDc = 50;
  /* Dual Bridge configuration  (FALSE for mono, TRUE for dual brush dc) */
  initVal.dualBridgeEnabled = TRUE;

  for (uint32_t id = 0; id < MAX_BDC_MOTORS; id++)
  {
    motorParam[id].motorId = id;
    motorParam[id].direction = BDCMotor::FWD;
  }

}

void MotorDriverStmBLDC::init()
{
  initStmBLDC(motor);
}

void MotorDriverStmBLDC::setDirection(uint32_t motorId, uint32_t direction)
{
  if(motorId < MAX_BDC_MOTORS)
  {
    motorParam[motorId].direction = static_cast<BDCMotor::direction_t>(direction);
  }
}

void MotorDriverStmBLDC::setSpeed(uint32_t motorId, uint32_t speed)
{
  if(0 != motor && motorId < MAX_BDC_MOTORS)
  {
    motor->set_speed(motorId, speed);
  }
}

void MotorDriverStmBLDC::run(uint32_t motorId)
{
  if(0 != motor && motorId < MAX_BDC_MOTORS)
  {
    motor->run(motorId, motorParam[motorId].direction);
  }
}

void MotorDriverStmBLDC::stop(uint32_t motorId, bool disableBridge)
{
  if(0 != motor && motorId < MAX_BDC_MOTORS)
  {
    if(disableBridge)
    {
      motor->hard_hiz(motorId);
    }
    else
    {
      motor->hard_stop(motorId);
    }
  }
}

void MotorDriverStmBLDC::initStmBLDC(STSpin240_250 *motor)
{
    /* Initializing Motor Control Component. */
  #if (defined TARGET_NUCLEO_F030R8)||(defined TARGET_NUCLEO_F334R8)
  motor = new STSpin240_250(D2, D9, D6, D7, D5, D4, A2);
  #elif (defined TARGET_NUCLEO_L152RE)
  motor = new STSpin240_250(D2, D9, D6, D7, D5, D4, A3);
  #elif (defined TARGET_NUCLEO_F429ZI)
  motor = new STSpin240_250(D2, D9, D6, D7, D5, D3, A0);
  #else
  motor = new STSpin240_250(D2, D9, D6, D7, D5, D4, A0);
  #endif
  if (motor->init(&initStmBLDC) != COMPONENT_OK) exit(EXIT_FAILURE);
}

MotorDriverStmBLDC::~MotorDriverStmBLDC()
{

}
