/*
 * pid.h
 *
 *  Created on: Feb 1, 2020
 *      Author: Vijay Katoch
 */

#ifndef PID_H_
#define PID_H_

#include<cstdint>

namespace KATBOT
{
  class PID
  {
    public:
      PID();
      PID(float _kp, float _ki, float _kd);
      void setSetPoint(float _setPoint);
      void setInput(float _input);
      void setTunings(float _kp, float _ki, float _kd);
      void setOutputLimits(float _min, float _max);
      void setSampleTimeinMS(std::uint32_t newSampleTime);
      void computeISR();
      float getKp();
      float getKi();
      float getKd();
      float getOutput();
      float getSetPoint();
      float getInput();
      std::uint32_t getSampleTimeinMS();
      void setMode(bool _active);
      void setControllerDirection(bool _direct);

    private:
      void reset();
      float kp, ki, kd;
      float input, setPoint, output;
      float Itotal, prevInput, prevError;
      float minLimit, maxLimit;
      std::uint32_t sampleTime;
      bool isAutoMode;
      bool isControllerDirectionDirect;
  };
}   /*PID_CTRL*/

#endif /* PID_H_ */
