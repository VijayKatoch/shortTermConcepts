/*
 * motor_pid_node.h
 *
 *  Created on: Feb 16, 2020
 *      Author: Vijay Katoch
 */

#ifndef MOTOR_PID_NODE_H_
#define MOTOR_PID_NODE_H_

#include <ros.h>

namespace KATBOT
{
  class MotorPIDNode
  {
    public:

    MotorPIDNode();

    void numberCallback();

    void encoderCallback();

    void pidCallback();

    private:

  };
}



#endif /* MOTOR_PID_NODE_H_ */
