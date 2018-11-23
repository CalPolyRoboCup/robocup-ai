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
  class DriveController : public controller_interface::Controller<hardware_interface::VelocityJointInterface>
  {
    public:
      bool init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle& h);
      void update(const ros::Time& time, const ros::Duration& period);
      void starting(const ros::Time& time);
      void stopping(const ros::Time& time);

    private:
      float x;
      float y;
      float angle;
      //std::vector<hardware_interface::JointHandle> drive_motors;
      std::vector<std::string> joint_names;
      int num_joints;
      hardware_interface::JointHandle motor_1;
      hardware_interface::JointHandle motor_2;
      hardware_interface::JointHandle motor_3;
      hardware_interface::JointHandle motor_4;
      ros::Subscriber pose_sub;
      void handleDriveCommand(const geometry_msgs::Pose pose);
  };
}
#endif /* !DRIVE_CONTROLLER_H */
