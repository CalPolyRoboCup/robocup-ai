/*
 * drive_controller.h
 * Copyright (C) 2018 willdle <willdle@willdle-ThinkPad-X1-Carbon>
 *
 * Distributed under terms of the MIT license.
 */

#ifndef DRIVE_CONTROLLER_H
#define DRIVE_CONTROLLER_H
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>

namespace robocup_control
{
  /**
   * @brief Class for implementing controller for RoboCup robots
   */
class DriveController : public controller_interface::Controller<hardware_interface::VelocityJointInterface>
{
public:
  /**
   * @brief Initializes drive controller
   * @param hw velocity joint interface that tracks the velocity of the motor
   * @param h  node handle for control thread
   * @return true if controller is successfully created, false otherwise
   */
  bool init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle& h);
  /**
   * @brief Updates the drive motor speeds
   * @param time ros::Time object to track time
   * @param period ros::Duration object to track update period
   */
  void update(const ros::Time& time, const ros::Duration& period);
  /**
   * @brief Determines the behavior of the controller upon starting
   * @param ros::Time object to track time
   */
  void starting(const ros::Time& time);
  /**
   * @brief Determines the behavior of the controller when it is stopped
   * @param ros::Time object to track time
   */
  void stopping(const ros::Time& time);

private:
  // x,y, theta velocities
  double x;
  double y;
  double angle;

  double update_rate;
  std::vector<hardware_interface::JointHandle> drive_motors;
  std::vector<double> motor_angles;
  std::vector<std::string> joint_names;
  int num_joints;
  ros::Subscriber pose_sub;
  // subscriber call back for new commanded x, y, theta velocities
  void handleDriveCommand(const geometry_msgs::Pose pose);
};
}
#endif /* !DRIVE_CONTROLLER_H */
