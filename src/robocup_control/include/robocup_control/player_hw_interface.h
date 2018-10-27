/*
 * player_hw_interface.h
 * Copyright (C) 2018 willdle <willdle@willdle-ThinkPad-X1-Carbon>
 *
 * Distributed under terms of the MIT license.
 */

#ifndef PLAYER_HW_INTERFACE_H
#define PLAYER_HW_INTERFACE_H
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include <ros/ros.h>

class PlayerHWInterface : public hardware_interface::RobotHW
{
  public:
    PlayerHWInterface(ros::NodeHandle nh);
    //~PlayerHWInterface();
    bool init();
    void read();
    void write();

  private:
    // hardware interfaces
    hardware_interface::JointStateInterface jnt_st_interface;
    hardware_interface::VelocityJointInterface jnt_vel_interface;
    
    // hardware handles
    hardware_interface::JointHandle motor_1;
    hardware_interface::JointHandle motor_2;
    hardware_interface::JointHandle motor_3;
    hardware_interface::JointHandle motor_4;

    //server clients for communicating
    ros::ServiceClient cmd_client;
    ros::ServiceClient data_client;


    //parameters for interface
    std::vector<std::string> joint_names;
    int num_joints;
    std::vector<double> cmd;
    std::vector<double> pos;
    std::vector<double> vel;
    std::vector<double> eff;
};
#endif /* !PLAYER_HW_INTERFACE_H */
