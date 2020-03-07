#ifndef _ROS_katbot_msgs_Motor01_Int32_h
#define _ROS_katbot_msgs_Motor01_Int32_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace katbot_msgs
{

  class Motor01_Int32 : public ros::Msg
  {
    public:
      typedef int32_t _motor0_type;
      _motor0_type motor0;
      typedef int32_t _motor1_type;
      _motor1_type motor1;

    Motor01_Int32():
      motor0(0),
      motor1(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_motor0;
      u_motor0.real = this->motor0;
      *(outbuffer + offset + 0) = (u_motor0.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_motor0.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_motor0.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_motor0.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->motor0);
      union {
        int32_t real;
        uint32_t base;
      } u_motor1;
      u_motor1.real = this->motor1;
      *(outbuffer + offset + 0) = (u_motor1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_motor1.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_motor1.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_motor1.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->motor1);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_motor0;
      u_motor0.base = 0;
      u_motor0.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_motor0.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_motor0.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_motor0.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->motor0 = u_motor0.real;
      offset += sizeof(this->motor0);
      union {
        int32_t real;
        uint32_t base;
      } u_motor1;
      u_motor1.base = 0;
      u_motor1.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_motor1.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_motor1.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_motor1.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->motor1 = u_motor1.real;
      offset += sizeof(this->motor1);
     return offset;
    }

    const char * getType(){ return "katbot_msgs/Motor01_Int32"; };
    const char * getMD5(){ return "0eccbb3d6a4de0bdd5e47bad3f68a6ee"; };

  };

}
#endif
