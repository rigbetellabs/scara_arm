#ifndef scara_arm_HW_INTERFACE_H
#define scara_arm_HW_INTERFACE_H

#include <ros_control_boilerplate/generic_hw_interface.h>
#include <scara_arm_hw_interface/joint_arm.h>

#define DEG_TO_RAD 0.01745329251
#define RAD_TO_DEG 57.2957795131

namespace scara_arm_ns
{
/** \brief Hardware interface for a robot */
class scara_armHWInterface : public ros_control_boilerplate::GenericHWInterface
{
public:
  /**
   * \brief Constructor
   * \param nh - Node handle for topics.
   */
  scara_armHWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model = NULL);

  /** \brief Initialize the robot hardware interface */
  virtual void init();

  /** \brief Read the state from the robot hardware. */
  virtual void read(ros::Duration& elapsed_time);

  /** \brief Write the command to the robot hardware. */
  virtual void write(ros::Duration& elapsed_time);

  /** \briesf Enforce limits for all values before writing */
  virtual void enforceLimits(ros::Duration& period);

protected:

ros::Publisher command_pub;

ros::Subscriber telemetry_sub;
void armTelemetryCallback(const scara_arm_hw_interface::joint_arm::ConstPtr &msg);

std::string ATTACHED_GRIPPER_NAME;
float END_EFF_MULTIPLIER;


};  // class

}  // namespace scara_arm_ns

#endif
