#ifndef _ROS_scara_arm_hw_interface_joint_arm_h
#define _ROS_scara_arm_hw_interface_joint_arm_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace scara_arm_hw_interface
{

  class joint_arm : public ros::Msg
  {
    public:
      typedef const char* _type_type;
      _type_type type;
      typedef float _joint_1_type;
      _joint_1_type joint_1;
      typedef float _joint_2_type;
      _joint_2_type joint_2;
      typedef float _z_axis_type;
      _z_axis_type z_axis;
      typedef float _joint_4_type;
      _joint_4_type joint_4;
      typedef float _gripper_type;
      _gripper_type gripper;

    joint_arm():
      type(""),
      joint_1(0),
      joint_2(0),
      z_axis(0),
      joint_4(0),
      gripper(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_type = strlen(this->type);
      varToArr(outbuffer + offset, length_type);
      offset += 4;
      memcpy(outbuffer + offset, this->type, length_type);
      offset += length_type;
      union {
        float real;
        uint32_t base;
      } u_joint_1;
      u_joint_1.real = this->joint_1;
      *(outbuffer + offset + 0) = (u_joint_1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_joint_1.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_joint_1.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_joint_1.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint_1);
      union {
        float real;
        uint32_t base;
      } u_joint_2;
      u_joint_2.real = this->joint_2;
      *(outbuffer + offset + 0) = (u_joint_2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_joint_2.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_joint_2.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_joint_2.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint_2);
      union {
        float real;
        uint32_t base;
      } u_z_axis;
      u_z_axis.real = this->z_axis;
      *(outbuffer + offset + 0) = (u_z_axis.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_z_axis.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_z_axis.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_z_axis.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->z_axis);
      union {
        float real;
        uint32_t base;
      } u_joint_4;
      u_joint_4.real = this->joint_4;
      *(outbuffer + offset + 0) = (u_joint_4.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_joint_4.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_joint_4.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_joint_4.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint_4);
      union {
        float real;
        uint32_t base;
      } u_gripper;
      u_gripper.real = this->gripper;
      *(outbuffer + offset + 0) = (u_gripper.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_gripper.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_gripper.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_gripper.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->gripper);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_type;
      arrToVar(length_type, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_type; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_type-1]=0;
      this->type = (char *)(inbuffer + offset-1);
      offset += length_type;
      union {
        float real;
        uint32_t base;
      } u_joint_1;
      u_joint_1.base = 0;
      u_joint_1.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_joint_1.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_joint_1.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_joint_1.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->joint_1 = u_joint_1.real;
      offset += sizeof(this->joint_1);
      union {
        float real;
        uint32_t base;
      } u_joint_2;
      u_joint_2.base = 0;
      u_joint_2.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_joint_2.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_joint_2.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_joint_2.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->joint_2 = u_joint_2.real;
      offset += sizeof(this->joint_2);
      union {
        float real;
        uint32_t base;
      } u_z_axis;
      u_z_axis.base = 0;
      u_z_axis.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_z_axis.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_z_axis.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_z_axis.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->z_axis = u_z_axis.real;
      offset += sizeof(this->z_axis);
      union {
        float real;
        uint32_t base;
      } u_joint_4;
      u_joint_4.base = 0;
      u_joint_4.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_joint_4.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_joint_4.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_joint_4.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->joint_4 = u_joint_4.real;
      offset += sizeof(this->joint_4);
      union {
        float real;
        uint32_t base;
      } u_gripper;
      u_gripper.base = 0;
      u_gripper.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_gripper.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_gripper.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_gripper.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->gripper = u_gripper.real;
      offset += sizeof(this->gripper);
     return offset;
    }

    virtual const char * getType() override { return "scara_arm_hw_interface/joint_arm"; };
    virtual const char * getMD5() override { return "8119cd182fde7e54cda8856243791299"; };

  };

}
#endif
