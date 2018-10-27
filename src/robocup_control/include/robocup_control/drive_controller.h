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
#include <pluginlib/class_list_marcos.h>
#include <geometry_msgs/Pose2D.h>
#include <QUDPSocket>

namespace drive_controller_ns
{
class DriveController : public controller::Controller<hardware_interface::JointVelocityInterface>
{
public:
  DriveController();
  bool init();
  void update(const ros::Time& time, const ros::Duration period);
  void starting(const ros::Time& time);
  void stopping(const ros::Time& time);

private:
  int16_t x;
  int16_t y;
  int8_t angle;
  void handleDriveCommand(const geometry_msgs::Pose2D pose);
};
PLUGINLIB_DECLARE_CLASS(package_name, DriveController, drive_controller_ns::DriveController,
                        controller_interface::ControllerBase);
}
#endif /* !DRIVE_CONTROLLER_H */
