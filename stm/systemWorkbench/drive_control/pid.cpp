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
  input = 0;
  output = 0;
  setPoint = 0;
  sampleTime = 1000;
  isAutoMode = true;
  isControllerDirectionDirect = true;

  setOutputLimits(0, 255);
  reset();
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
  if(_kp<0 || _ki<0 || _kd<0) return;

  float sampleTimeinSec = ((float)sampleTime)/1000;
  kp = _kp;
  ki = _ki * sampleTimeinSec;
  kd = _kd / sampleTimeinSec;

  if(!isControllerDirectionDirect)
  {
    kp = (0 - kp);
    ki = (0 - ki);
    kd = (0 - kd);
  }


}

void PID::setOutputLimits(float _min, float _max)
{
  if(_min > _max)
    return;

  minLimit = _min;
  maxLimit = _max;
}

void PID::setSampleTimeinMS(uint32_t newSampleTime)
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
  if(!isAutoMode) return;

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

void PID::setMode(bool _auto)
{
  if(!isAutoMode && _auto)
  {
    /* We changed from Manual to Auto mode, reset once */
    reset();
  }
  isAutoMode = _auto;
}

void PID::setControllerDirection(bool _direct)
{
  isControllerDirectionDirect = _direct;
}

void PID::reset()
{
  Itotal = output;
  prevInput = input;
  prevError = 0;

  Itotal = min(Itotal, maxLimit);
  Itotal = max(minLimit, Itotal);
}
