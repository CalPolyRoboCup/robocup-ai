/*
 * drive_controller.cpp
 * Copyright (C) 2018 willdle <willdle@willdle-ThinkPad-X1-Carbon>
 *
 * Distributed under terms of the MIT license.
 */

#include <robocup_control/drive_controller.h>

namespace robocup_control
{
  bool DriveController::init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle& n)
  {
    ROS_INFO("Initializing drive controller");
    n.getParam("/robocup_control/drive_controller/joints", joint_names);

    num_joints = joint_names.size();
    for(int i = 0; i < num_joints; i++)
    {
      motor_1 = hw->getHandle(joint_names[0]);
      motor_2 = hw->getHandle(joint_names[1]);
      motor_3 = hw->getHandle(joint_names[2]);
      motor_4 = hw->getHandle(joint_names[3]);
    }
  
    // subcribing to pose commands
    // will only be switched on for simulation
    // need a different secure and reliable protocol
    pose_sub = n.subscribe("/robocup_control/command_pose", 1, &DriveController::handleDriveCommand, this);

    return true;
  }

  void DriveController::update(const ros::Time& time, const ros::Duration& period)
  {
    ROS_INFO("Running controller update");
    float motor_speed_1 = sqrt(2) / 2 * x + sqrt(2) / 2 * y + angle;
    float motor_speed_2 = sqrt(2) / 2 * x + sqrt(2) / 2 * y + angle;
    float motor_speed_3 = sqrt(2) / 2 * x + sqrt(2) / 2 * y + angle;
    float motor_speed_4 = sqrt(2) / 2 * x + sqrt(2) / 2 * y + angle;
    ROS_INFO("Commanded speeds: %f, %f, %f, %f", motor_speed_1, motor_speed_2, motor_speed_3, motor_speed_4);
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

  void DriveController::handleDriveCommand(const geometry_msgs::Pose pose)
  {
    ROS_INFO("Received callback");
    this->x = pose.position.x;
    this->y = pose.position.y;
    this->angle = pose.position.z;
  }
}
PLUGINLIB_EXPORT_CLASS(robocup_control::DriveController, controller_interface::ControllerBase)
