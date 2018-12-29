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
    n.getParam("/robocup_control/drive_controller/joints", joint_names);
    //get publish rate here
    n.getParam("/robocup_control/joint_state_controller/publish_rate", pub_rate);
    update_rate = 1/pub_rate;

    //save the publish rate in a variable to get the right update time
    
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

  double DriveController::saturatedAdd(double x1, double x2, double max)
  {
    if((x1+x2) < max)
    {
      return x1+x2;
    }
    return max;
  }

  double DriveController::calcPID(hardware_interface::JointHandle handle, double command, double p, double i, double d)
  {
    double error;
    static double last_error;
    static double sum_error;
    static double last_cmd;
    double diff_error;
    double cmd;

    double k_i = i * update_rate;
    double k_d = d / update_rate;
    double set_pt_change = last_cmd - command;
    
    //ROS_INFO("I: %f, D: %f, delta: %f", k_i, k_d, set_pt_change);
    ROS_INFO("Current Velocity: %f", handle.getVelocity());
    error = last_cmd - handle.getVelocity();
    ROS_INFO("Current Error: %f", error);
    sum_error = saturatedAdd(sum_error, k_i*error, 10000);
    ROS_INFO("Total Error: %f", sum_error);
    diff_error = last_error - error;
    diff_error = set_pt_change - diff_error;
    ROS_INFO("Change in error: %f", diff_error);
    cmd = p * error + k_i * sum_error + k_d * diff_error;

    last_cmd = command;
    last_error = error;
    return cmd;
  }

  void DriveController::update(const ros::Time& time, const ros::Duration& period)
  {
    ROS_INFO("Running controller update");
    double pi = 3.14159265;
    /*
     * lol, the way the dynamics should work out
    double motor_speed_1 = ((sqrt(2) / -2) * x) + ((sqrt(2) / 2) * y) + angle;
    double motor_speed_2 = ((sqrt(2) / 2) * x) + ((sqrt(2) / 2) * y) + angle;
    double motor_speed_3 = ((sqrt(2) / 2) * x) + ((sqrt(2) / -2) * y) + angle;
    double motor_speed_4 = ((sqrt(2) / -2) * x) + ((sqrt(2) / -2) * y) + angle;
    */
    // literally what grSim did
    // (1.0 / cfg->robotSettings.WheelRadius) * (( (cfg->robotSettings.RobotRadius * vw) - (vx * sin(motorAlpha[0])) + (vy * cos(motorAlpha[0]))) );
    double motor_speed_1 = (1.0 / 0.0325) * ((0.09 * angle) - ( x * sin(pi/3)) + (y * cos(pi/3)));
    double motor_speed_2 = (1.0 / 0.0325) * ((0.09 * angle) - ( x * sin((3*pi)/4)) + (y * cos((3*pi)/4)));
    double motor_speed_3 = (1.0 / 0.0325) * ((0.09 * angle) - ( x * sin((5*pi)/4)) + (y * cos((5*pi)/4)));
    double motor_speed_4 = (1.0 / 0.0325) * ((0.09 * angle) - ( x * sin((5*pi)/3)) + (y * cos((5*pi)/3)));;
    ROS_INFO("Commanded speeds: %lf, %lf, %lf, %lf", motor_speed_1, motor_speed_2, motor_speed_3, motor_speed_4);
    //ROS_INFO("Radius scaling: %lf, Angular component: %lf, X component: %lf, Y component: %lf", (1.0/0.0325), (0.09*angle), (x * sin(pi/3)), (y * cos(pi/3)));
    
    /*ROS_INFO("Motor 1 Calc");
    motor_speed_1 = calcPID(motor_1, motor_speed_1, 6.0, 1.0, 0.0);
    ROS_INFO("Motor 2 Calc");
    motor_speed_2 = calcPID(motor_2, motor_speed_2, 6.0, 1.0, 0.0);
    ROS_INFO("Motor 3 Calc");
    motor_speed_3 = calcPID(motor_3, motor_speed_3, 6.0, 1.0, 0.0);
    ROS_INFO("Motor 4 Calc");
    motor_speed_4 = calcPID(motor_4, motor_speed_4, 6.0, 1.0, 0.0);

    ROS_INFO("Controlled speeds: %f, %f, %f, %f", motor_speed_1, motor_speed_2, motor_speed_3, motor_speed_4);*/
    
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
    //ROS_INFO("Received callback");
    this->x = pose.position.x;
    this->y = pose.position.y;
    this->angle = pose.position.z;
  }
}
PLUGINLIB_EXPORT_CLASS(robocup_control::DriveController, controller_interface::ControllerBase)
