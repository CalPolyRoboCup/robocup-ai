/*
 * player_hw_interface.cpp
 * Copyright (C) 2018 willdle <willdle@willdle-ThinkPad-X1-Carbon>
 *
 * Distributed under terms of the MIT license.
 */

#include <robocup_control/player_hw_interface.h>
#include <robocup_control/Insn.h>
#include <robocup_control/Data.h>
using namespace std;
PlayerHWInterface::PlayerHWInterface(ros::NodeHandle nh)
{
  cout << "Initializng Player Hardware Interface" << endl;
  nh.getParam("/player/drive_controller/joints", joint_names);
  cout << "Done getting joint parameters" << endl;
  num_joints = joint_names.size();
  cmd.resize(num_joints);
  pos.resize(num_joints);
  vel.resize(num_joints);
  eff.resize(num_joints);
  cout << "Resized joints" << endl;
  cout << "Number of joints: " << num_joints << endl;
  for(int i = 0; i < num_joints; i++)
  {
    cout << "Registering joint to joint state interface: " << joint_names[i] << endl;
    //connect and register the joint state interface
    hardware_interface::JointStateHandle jnt_st_handle(joint_names[i], &pos[i], &vel[i], &eff[i]);
    jnt_st_interface.registerHandle(jnt_st_handle);
    cout << "Registering joint to joint velocity interface: " << joint_names[i] << endl;
    //connect and register the joint velocity interface
    hardware_interface::JointHandle jnt_vel_handle(jnt_st_interface.getHandle(joint_names[i]), &cmd[i]);
    jnt_vel_interface.registerHandle(jnt_vel_handle);
  }

  cout << "Registering interfaces" << endl;
  registerInterface(&jnt_st_interface);
  registerInterface(&jnt_vel_interface);

  cout << "Getting motor handles" << endl;
  motor_1 = jnt_vel_interface.getHandle(joint_names[0]);
  motor_2 = jnt_vel_interface.getHandle(joint_names[1]);
  motor_3 = jnt_vel_interface.getHandle(joint_names[2]);
  motor_4 = jnt_vel_interface.getHandle(joint_names[3]);

  cmd_client = nh.serviceClient<robocup_control::Insn>("player_insns");
  data_client = nh.serviceClient<robocup_control::Data>("player_data");
}

/*PlayerHWInterface::~PlayerHWInterface()
{
  delete &jnt_st_interface, &jnt_vel_interface, &cmd_client, &data_client;
}*/

void PlayerHWInterface::read()
{
    robocup_control::Data plz;
    plz.request.plz = true;
    if (data_client.call(plz))
    {
      ROS_INFO("Receiving data");
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
