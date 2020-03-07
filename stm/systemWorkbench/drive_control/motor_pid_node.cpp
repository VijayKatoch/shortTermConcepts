/*
 * motor_pid_node.cpp
 *
 *  Created on: Feb 16, 2020
 *      Author: Vijay Katoch
 */

#include "motor_pid_node.h"

using namespace KATBOT;

MotorPIDNode::MotorPIDNode() :targetSub("cmd_target", &MotorPIDNode::targetCallback,this),
    encoderSub("rpm", &MotorPIDNode::encoderCallback,this)
{  
  
  
  
}

void MotorPIDNode::targetCallback(const 
katbot_msgs::Motor01_Float32 &msg)
{

}

void MotorPIDNode::encoderCallback(const 
katbot_msgs::Motor01_Int32 &msg)
{

}

/*
void MotorPIDNode::pidCallback()
{

}
*/
