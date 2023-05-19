#include <scara_arm_hw_interface/scara_arm_hw_interface.h>

namespace scara_arm_ns
{
scara_armHWInterface::scara_armHWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model)
  : ros_control_boilerplate::GenericHWInterface(nh, urdf_model)
{
  telemetry_sub= nh.subscribe("/arduino/armTelemetry",1,&scara_armHWInterface::armTelemetryCallback,this);

  command_pub = nh.advertise<scara_arm_hw_interface::joint_arm>("/arduino/armCmd",1);
  ROS_INFO("scara_armHWInterface constructed!");

  if(nh.getParam("/scara_arm/attached_gripper", ATTACHED_GRIPPER_NAME)){
    ROS_INFO_STREAM("HW INTERFACE: " << "THE GRIPPER NAME SET AS: " << ATTACHED_GRIPPER_NAME);
  } else {
    ROS_ERROR_STREAM("HW INTERFACE: " << "FAILED TO GET THE GRIPPER NAME");
  }
  std::string GRIPPER_PARAM = "/scara_arm_grippers/" + ATTACHED_GRIPPER_NAME + "/multiplier";

  if(nh.getParam(GRIPPER_PARAM, END_EFF_MULTIPLIER)){
    ROS_INFO_STREAM("HW INTERFACE: " << "THE GRIPPER MULTIPLIER SET AS: " << END_EFF_MULTIPLIER);
  } else {
    ROS_ERROR_STREAM("HW INTERFACE: " << "FAILED TO GET THE GRIPPER MULTIPLIER");
  }

}

void scara_armHWInterface::armTelemetryCallback(const scara_arm_hw_interface::joint_arm::ConstPtr &msg){
  for(int joint_num = 0; joint_num < joint_position_.size(); joint_num++){
    switch (joint_num)
    {
    case 0:
      joint_position_[joint_num] = msg->joint_1 * DEG_TO_RAD;
      break;

    case 1:
      joint_position_[joint_num] = msg->joint_2 * DEG_TO_RAD;
      break;
    case 2:
      joint_position_[joint_num] = msg->z_axis;
      break;
    case 3:
      joint_position_[joint_num] = msg->joint_4 * DEG_TO_RAD;
      break;
    case 4:
      joint_position_[joint_num] = msg->gripper * (1 / END_EFF_MULTIPLIER);
      break;
    
    default:
      break;
    }

    // if(joint_num == 2){
    //   joint_position_[joint_num] = msg->data[joint_num];
    //   continue;
    // }
    // joint_position_[joint_num] = msg->data[joint_num] * DEG_TO_RAD;


  }
}

void scara_armHWInterface::init()
{
  // Call parent class version of this function
  GenericHWInterface::init();
  ROS_INFO("scara_armHWInterface Ready.");
}

void scara_armHWInterface::read(ros::Duration& elapsed_time)
{
  ros::spinOnce();
}

void scara_armHWInterface::write(ros::Duration& elapsed_time)
{
  scara_arm_hw_interface::joint_arm cmd_msg;
  cmd_msg.type = "Command";

  // Safety
  enforceLimits(elapsed_time);

  for (int i = 0; i < joint_position_command_.size(); i++){
    switch (i)
    {
    case 0:
      cmd_msg.joint_1 = joint_position_command_[i] * RAD_TO_DEG;
      break;

    case 1:
      cmd_msg.joint_2 = joint_position_command_[i] * RAD_TO_DEG;
      break;
    case 2:
      cmd_msg.z_axis = joint_effort_command_[i];
      joint_effort_[i] = joint_effort_command_[i];
      break;
    case 3:
      cmd_msg.joint_4 = joint_position_command_[i] * RAD_TO_DEG;
      break;
    case 4:
      cmd_msg.gripper = joint_position_command_[i] * END_EFF_MULTIPLIER;
      break;
    
    default:
      break;
    }

    // if (i == 2){
    //   cmd_msg.data[i] = joint_effort_command_[i];
    //   joint_effort_[i] = joint_effort_command_[i];
    //   continue;
    // }
    // cmd_msg.data[i] = joint_position_command_[i] * RAD_TO_DEG;
  }
  command_pub.publish(cmd_msg);

} //write ends

void scara_armHWInterface::enforceLimits(ros::Duration& period)
{
  // Enforces position and velocity
  pos_jnt_sat_interface_.enforceLimits(period);
  eff_jnt_sat_interface_.enforceLimits(period);
}


}  // namespace scara_arm_ns