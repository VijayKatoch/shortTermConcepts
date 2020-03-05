/*
 * motor_pid_node.h
 *
 *  Created on: Feb 16, 2020
 *      Author: Vijay Katoch
 */

#ifndef MOTOR_PID_NODE_H_
#define MOTOR_PID_NODE_H_

#include <ros.h>
#include <ros/time.h>
#include "pid.h"

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
/*
    ros::Subscriber numberSub, encoderSub;
    ros::Publisher motorPub;
    ros::ServiceServer tuningService;

    ros::NodeHandle nh, _nh;
    ros::Timer controllerTimer;

 */   PID motor0RPMController, motor1RPMController;


  };
}



#endif /* MOTOR_PID_NODE_H_ */
