#include <scara_arm_hw_interface/scara_arm_hw_interface.h>

namespace scara_arm_ns
{
scara_armHWInterface::scara_armHWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model)
  : ros_control_boilerplate::GenericHWInterface(nh, urdf_model)
{
  effort_pub = nh.advertise<std_msgs::Float32>("/arduino_arm_command",1);
  ROS_INFO("scara_armHWInterface constructed!");

}

void scara_armHWInterface::init()
{
  // Call parent class version of this function
  GenericHWInterface::init();
  ROS_INFO("scara_armHWInterface Ready.");
}

void scara_armHWInterface::read(ros::Duration& elapsed_time)
{

}

void scara_armHWInterface::write(ros::Duration& elapsed_time)
{
  // Safety
  enforceLimits(elapsed_time);
  for (size_t i = 0; i < joint_position_command_.size(); i++){
    ROS_INFO_STREAM("Joint_pose_command_ for Joint "<< i << " "<< joint_position_command_[i]);
    joint_position_[i] = joint_position_command_[i];
  }
  

  for (size_t i = 0; i < joint_position_command_.size(); i++){
    ROS_INFO_STREAM("Joint_effort_command_ for Joint "<< i << " "<< joint_effort_command_[0]);
    joint_effort_[i] = joint_effort_command_[i];
  }
  std_msgs::Float32 msg;
  msg.data = joint_effort_command_[2];
  effort_pub.publish(msg);

} //write ends

void scara_armHWInterface::enforceLimits(ros::Duration& period)
{
  // Enforces position and velocity
  pos_jnt_sat_interface_.enforceLimits(period);
  // eff_jnt_sat_interface_.enforceLimits(period);
}


}  // namespace scara_arm_ns