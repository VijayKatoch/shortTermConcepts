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
#include <katbot_msgs/Motor01_Float32.h>
#include <katbot_msgs/Motor01_Int32.h>
#include "pid.h"

namespace KATBOT
{
  class MotorPIDNode
  {
    public:

    MotorPIDNode();

    void targetCallback(const 
    katbot_msgs::Motor01_Float32 &msg);

    void encoderCallback(const 
    katbot_msgs::Motor01_Int32 &msg);

    //void pidCallback();

    private:
    
    /*!< subscriber for commanded pid target */
    ros::Subscriber<katbot_msgs::Motor01_Float32, MotorPIDNode> targetSub;
    /*!< subscriber for encoder as pid input */    
    ros::Subscriber<katbot_msgs::Motor01_Int32, MotorPIDNode> encoderSub;   
    //ros::Publisher motorPub;      /*!< publish pid output e.g motor rpm msg */
    //ros::ServiceServer tuningService; /*!< tuning service for kp, ki, kd*/

    ros::NodeHandle nh, _nh;
    //ros::Timer controllerTimer;   /*!< timer to call pid compute method*/

    PID motor0RPMController, motor1RPMController;

  };
}



#endif /* MOTOR_PID_NODE_H_ */
