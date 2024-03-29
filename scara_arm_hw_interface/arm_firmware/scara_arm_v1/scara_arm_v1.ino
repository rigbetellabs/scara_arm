/*
   yellow signal
   green vcc center
   orange ground
*/


#include <Servo.h>
#include <ros.h>
#include "scara_arm_hw_interface/joint_arm.h"

#define SERVO_COUNT 5
ros::NodeHandle  nh;
scara_arm_hw_interface::joint_arm armTelemetry_msg;
ros::Publisher armTelemetry("/arduino/armTelemetry", &armTelemetry_msg);

size_t servo_pin[SERVO_COUNT] = { 3, 5, 6, 12, 10};

Servo servo[SERVO_COUNT];
int servo_val[SERVO_COUNT];

void armCmd_cb(const scara_arm_hw_interface::joint_arm& cmd_msg) {

  for (size_t joint_num = 0; joint_num < 5; joint_num++) {
    switch (joint_num)
    {
      case 0:
        armTelemetry_msg.joint_1 = (float)cmd_msg.joint_1;
        servo_val[joint_num] = (int)map(cmd_msg.joint_1, -82, 98, 0, 180);
        servo[joint_num].write(servo_val[joint_num]);
        break;
      case 1:
        armTelemetry_msg.joint_2 = (float)cmd_msg.joint_2;
        servo_val[joint_num] = (int)map(cmd_msg.joint_2, -98, 82, 0, 180);
        servo[joint_num].write(servo_val[joint_num]);
        break;
      case 3:
        armTelemetry_msg.joint_4 = (float)cmd_msg.joint_4;
        servo_val[joint_num] = (int)map(cmd_msg.joint_4, -90, 90, 0, 180);
        servo[joint_num].write(servo_val[joint_num]);
        break;
      case 4:
        armTelemetry_msg.gripper = (float)cmd_msg.gripper;
        servo_val[joint_num] = (int)(cmd_msg.gripper);
        servo[joint_num].write(servo_val[joint_num]);
        break;
      case 2:
        int sensorValue = analogRead(A0);
        armTelemetry_msg.z_axis = (float)((sensorValue - 575.0) * (-0.24 - 0.0) / (795.0 - 575.0) + 0.0);
        servo_val[joint_num] = (int)map(cmd_msg.z_axis, -100, 100, 180, 0);
        servo[joint_num].write(servo_val[joint_num]);
        break;
      default:
        break;
    }
  }
  armTelemetry_msg.type = "Telemetry";
  armTelemetry.publish(&armTelemetry_msg);
}

ros::Subscriber<scara_arm_hw_interface::joint_arm> armCmd("/arduino/armCmd", armCmd_cb);


void setup() {

  for (size_t i = 0; i < SERVO_COUNT; i++) {
    servo[i].attach(servo_pin[i]);
  }
  pinMode(A0, INPUT);
  nh.initNode();
  nh.subscribe(armCmd);
  nh.advertise(armTelemetry);
}

void loop() {
  nh.spinOnce();
  delay(10);
}
