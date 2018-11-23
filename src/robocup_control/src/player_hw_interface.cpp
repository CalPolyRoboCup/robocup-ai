/*
 * player_hw_interface.cpp
 * Copyright (C) 2018 willdle <willdle@willdle-ThinkPad-X1-Carbon>
 *
 * Distributed under terms of the MIT license.
 */

#include <robocup_control/player_hw_interface.h>
#include <robocup_control/Insn.h>
#include <robocup_control/Data.h>
namespace robocup_control
{
  PlayerHWInterface::PlayerHWInterface(ros::NodeHandle nh)
  {
    ROS_INFO("Initializng Player Hardware Interface");
    nh.getParam("/robocup_control/drive_controller/joints", joint_names);
    ROS_INFO("Done getting joint parameters");
  
    num_joints = joint_names.size();
      
    cmd_client = nh.serviceClient<robocup_control::Insn>("player_insns");
    data_client = nh.serviceClient<robocup_control::Data>("player_data");
  }

  bool PlayerHWInterface::init()
  {
    vel_cmd.resize(num_joints);
    pos.resize(num_joints);
    vel.resize(num_joints);
    eff.resize(num_joints);
  
    ROS_INFO("Resized joints");
    ROS_INFO("Number of joints: %zd", num_joints);
  
    for(int i = 0; i < num_joints; i++)
    {
      ROS_INFO("Registering joint to joint state interface: %s", joint_names[i].c_str());
      //connect and register the joint state interface
      hardware_interface::JointStateHandle jnt_st_handle(joint_names[i], &pos[i], &vel[i], &eff[i]);
      jnt_st_interface.registerHandle(jnt_st_handle);

      ROS_INFO("Registering joint to joint velocity interface: %s",joint_names[i].c_str());
      //connect and register the joint velocity interface
      hardware_interface::JointHandle jnt_vel_handle(jnt_st_interface.getHandle(joint_names[i]), &vel_cmd[i]);
      jnt_vel_interface.registerHandle(jnt_vel_handle);
    }

    ROS_INFO("Registering interfaces");
    registerInterface(&jnt_st_interface);
    registerInterface(&jnt_vel_interface);

    ROS_INFO("Getting motor handles");
    motor_1 = jnt_vel_interface.getHandle(joint_names[0]);
    motor_2 = jnt_vel_interface.getHandle(joint_names[1]);
    motor_3 = jnt_vel_interface.getHandle(joint_names[2]);
    motor_4 = jnt_vel_interface.getHandle(joint_names[3]);
    return true;
  }

  void PlayerHWInterface::read()
  {
    robocup_control::Data plz;
    plz.request.plz = true;
    if (data_client.call(plz))
    {
      ROS_INFO("Receiving data");
      ROS_INFO("Motor 1: %f, Motor 2: %f, Motor 3: %f, Motor 4: %f", vel[0], vel[1], vel[2], vel[3]);
      //vel[0] = 50.0;
      //vel[1] = 150.0;
      //vel[2] = 250.0;
      //vel[3] = 350.0;
      return;
    }
    else
    {
      ROS_ERROR("Error communicating with robot\n");
    }
  }

  void PlayerHWInterface::write()
  {
    robocup_control::Insn insn;
    insn.request.motor1 = (uint16_t) motor_1.getCommand();
    insn.request.motor2 = (uint16_t) motor_2.getCommand();
    insn.request.motor3 = (uint16_t) motor_3.getCommand(); 
    insn.request.motor4 = (uint16_t) motor_4.getCommand();
    insn.request.kick = true;
    insn.request.dribble = true;
    if (cmd_client.call(insn))
    {
      ROS_INFO("Sending commands");
      return;
    }
    else
    {
      ROS_ERROR("Error communicating with robot\n");
    }
  }
}
