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
  double pub_rate = 0.0f;
  
  // effectively a constant
  double pi = 3.14159265;
  
  // load in the joint names from param server
  // http://wiki.ros.org/Parameter%20Server
  n.getParam("/robocup_control/drive_controller/joints", joint_names);
  
  // get publish rate here
  n.getParam("/robocup_control/joint_state_controller/publish_rate", pub_rate);
  update_rate = 1 / pub_rate;

  // save the publish rate in a variable to get the right update time
  num_joints = joint_names.size();
  drive_motors.resize(num_joints);

  // add angles to angle array
  motor_angles.push_back(pi / 3);
  motor_angles.push_back((3 * pi) / 4);
  motor_angles.push_back((5 * pi) / 4);
  motor_angles.push_back((5 * pi) / 3);

  for (int i = 0; i < num_joints; i++)
  {
    drive_motors[i] = hw->getHandle(joint_names[i]);
  }

  // subcribing to speed commands
  // will only be switched on for simulation
  speed_sub = n.subscribe("/robocup_control/command_speed", 1, &DriveController::handleDriveCommand, this);

  return true;
}

void DriveController::update(const ros::Time& time, const ros::Duration& period)
{
  ROS_INFO("Running controller update");

  double motor_speeds[4];
  for (int i = 0; i < num_joints; i++)
  {
    // literally, exactly what grSim did, lol
    motor_speeds[i] = (1.0 / 0.0325) * ((0.09 * angle) - (x * sin(motor_angles[i])) + (y * cos(motor_angles[i])));
    drive_motors[i].setCommand(motor_speeds[i]);
  }
}

void DriveController::starting(const ros::Time& time)
{
  for (int i = 0; i < num_joints; i++)
  {
    drive_motors[i].setCommand(0.0);
  }
}

void DriveController::stopping(const ros::Time& time)
{
  for (int i = 0; i < num_joints; i++)
  {
    drive_motors[i].setCommand(0.0);
  }
}

void DriveController::handleDriveCommand(const geometry_msgs::Twist speed)
{
  this->x = speed.linear.x;
  this->y = speed.linear.y;
  this->angle = speed.angular.z;
}
}
PLUGINLIB_EXPORT_CLASS(robocup_control::DriveController, controller_interface::ControllerBase)
