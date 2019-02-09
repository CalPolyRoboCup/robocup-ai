#ifndef PLAYER_HW_INTERFACE_H
#define PLAYER_HW_INTERFACE_H
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include <ros/ros.h>

/**
 * @brief Hardware interface for RoboCup robots
 */
namespace robocup_control
{
class PlayerHWInterface : public hardware_interface::RobotHW
{
public:
  /**
   * @brief Constructor
   * @param nh - Node handle for topics and services
   */
  PlayerHWInterface(ros::NodeHandle nh);

  /** @brief Destructor */
  virtual ~PlayerHWInterface()
  {
  }

  /** @brief Initalize interface */
  bool init();

  /** @brief Read robot joint states */
  void read();

  /** @brief Write new robot joint states */
  void write();

protected:
  // robot name
  std::string name;
  // hardware interfaces
  hardware_interface::JointStateInterface jnt_st_interface;
  hardware_interface::VelocityJointInterface jnt_vel_interface;

  // hardware handles
  hardware_interface::JointHandle motor_1;
  hardware_interface::JointHandle motor_2;
  hardware_interface::JointHandle motor_3;
  hardware_interface::JointHandle motor_4;

  // server clients for communicating
  ros::ServiceClient cmd_client;
  ros::ServiceClient data_client;

  // parameters for interface
  std::vector<std::string> joint_names;
  std::size_t num_joints;

  // joint variables
  std::vector<double> pos;
  std::vector<double> vel;
  std::vector<double> eff;

  // joint commands
  std::vector<double> vel_cmd;
};
}
#endif /* !PLAYER_HW_INTERFACE_H */
