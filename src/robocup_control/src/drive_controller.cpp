/*
 * drive_controller.cpp
 * Copyright (C) 2018 willdle <willdle@willdle-ThinkPad-X1-Carbon>
 *
 * Distributed under terms of the MIT license.
 */

#include <insn_controllers/drive_controller.h>

DriveController::DriveController()
{
}

bool DriveController::init(hardware_interface::VelocityJointInterface* hw, ros::Node_Handle& n)
{
  // defining the joints
  std::string motor_joint_1;
  std::string motor_joint_2;
  std::string motor_joint_3;
  std::string motor_joint_4;

  // getting the joint parameters
  if (!n.getParam("motor_1", motor_joint_1))
  {
    ROS_ERROR("Could not find joint");
    return false;
  }

  if (!n.getParam("motor_2", motor_joint_2))
  {
    ROS_ERROR("Could not find joint");
    return false;
  }

  if (!n.getParam("motor_3", motor_joint_3))
  {
    ROS_ERROR("Could not find joint");
    return false;
  }

  if (!n.getParam("motor_4", motor_joint_4))
  {
    ROS_ERROR("Could not find joint");
    return false;
  }

  // forming the handles
  motor_1 = hw->getHandle(motor_joint_1);
  motor_2 = hw->getHandle(motor_joint_2);
  motor_3 = hw->getHandle(motor_joint_3);
  motor_4 = hw->getHandle(motor_joint_4);

  // subcribing to pose commands
  pose_sub = n.subscribe("command_pose", 1, &DriveController::handleDriveCommand, this);

  return true;
}

void DriveController::update(const ros::Time& time, const ros::Duration& period)
{
  motor_speed_1 = sqrt(2) / 2 * x + sqrt(2) / 2 * y + angle;
  motor_speed_2 = sqrt(2) / 2 * x + sqrt(2) / 2 * y + angle;
  motor_speed_3 = sqrt(2) / 2 * x + sqrt(2) / 2 * y + angle;
  motor_speed_4 = sqrt(2) / 2 * x + sqrt(2) / 2 * y + angle;
  motor_1.setCommand(motor_speed_1);
  motor_2.setCommand(motor_speed_2);
  motor_3.setCommand(motor_speed_3);
  motor_4.setCommand(motor_speed_4);
}

void DriveController::starting(const ros::Time& time)
{
  motor_1.setCommand(0.0);
  motor_2.setCommand(0.0);
  motor_3.setCommand(0.0);
  motor_4.setCommand(0.0);
}

void DriveController::stopping(const ros::Time& time)
{
  motor_1.setCommand(0.0);
  motor_2.setCommand(0.0);
  motor_3.setCommand(0.0);
  motor_4.setCommand(0.0);
}

void DriveController::handleDriveCommand(const geometry_msg::Pose2D pose)
{
  this.x = pose.x;
  this.y = pose.y;
  this.angle = pose.angle;
}
