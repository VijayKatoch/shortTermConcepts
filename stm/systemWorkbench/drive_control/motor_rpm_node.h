/*
 * motor_rpm_node.h
 *
 *  Created on: Feb 16, 2020
 *      Author: Vijay Katoch
 */

#ifndef MOTOR_RPM_NODE_H_
#define MOTOR_RPM_NODE_H_

#include <ros.h>

namespace KATBOT
{
  class MotorRPMNode
  {

    public:

      MotorRPMNode();

      void encoderCallback();

      float rpmFromEncoderCount();

    private:
 /*
      ros::Subscriber encoderSub;
      ros::Publisher rpmPub;
      ros::NodeHandle nh, _nh;
*/
  };
}


#endif /* MOTOR_RPM_NODE_H_ */
