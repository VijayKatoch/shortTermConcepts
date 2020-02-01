/*
 * pid.cpp
 *
 *  Created on: Feb 1, 2020
 *      Author: Vijay Katoch
 */

#include <algorithm>

#include "pid.h"

using namespace std;
using namespace KATBOT;

PID::PID(float _kp, float _ki, float _kd): kp(_kp), ki(_ki), kd(_kd)
{
  reset();
  output = 0;
  input = 0;
  sampleTime = 1000;

  setOutputLimits(0, 255);
}

PID::PID()
{
  reset();
}

void PID::setSetPoint(float _setPoint)
{
  setPoint = _setPoint;
}

void PID::setInput(float _input)
{
  input = _input;
}

/**
  * @brief  Tune PID gain parameters
  * @param  [in] _kp kp value
  * @param  [in] _ki ki value in per seconds e.g. 1.0/sec
  * @param  [in] _kd kd value in per seconds e.g. 1.0/sec
  * @retval None.
  */
void PID::setTunings(float _kp, float _ki, float _kd)
{
  float sampleTimeinSec = ((float)sampleTime)/1000;
  kp = _kp;
  ki = _ki * sampleTimeinSec;
  kd = _kd / sampleTimeinSec;

  //reset();
}

void PID::setOutputLimits(float _min, float _max)
{
  if(_min > _max)
    return;

  minLimit = _min;
  maxLimit = _max;
}

void PID::setSampleTimeinMS(std::uint32_t newSampleTime)
{
  if(newSampleTime > 0)
  {
    float ratio = (float)newSampleTime / (float)sampleTime;

    ki *= ratio;
    kd /= ratio;
    sampleTime = newSampleTime;
  }
}

void PID::computeISR()
{
  float error = setPoint - input;

  Itotal += error*ki;
  Itotal = min(Itotal, maxLimit);
  Itotal = max(minLimit, Itotal);

  float dValue = kd * (input -prevInput);

  /*PID calculations*/
  output = kp * error + Itotal - dValue;

  /*Clamp output to bounds*/
  output = min(output, maxLimit);
  output = max(minLimit, output);

  prevInput = input;
  prevError = error;
}

float PID::getKp()
{
  return kp;
}

float PID::getKi()
{
  return ki;
}

float PID::getKd()
{
  return kd;
}

float PID::getOutput()
{
  return output;
}

float PID::getSetPoint()
{
  return setPoint;
}

float PID::getInput()
{
  return input;
}

uint32_t PID::getSampleTimeinMS()
{
  return sampleTime;
}

void PID::setActive(bool _active)
{
  if(!active && _active)
    reset();
  active = _active;
}

void PID::reset()
{
  Itotal = output;
  prevInput = 0;
  prevError = 0;

  Itotal = min(Itotal, maxLimit);
  Itotal = max(minLimit, Itotal);
}
