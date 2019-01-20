#include <robocup_control/player_hw_interface.h>
#include <robocup_control/Insn.h>
#include <robocup_control/Data.h>
namespace robocup_control
{
PlayerHWInterface::PlayerHWInterface(ros::NodeHandle nh)
{
  nh.getParam("/robocup_control/drive_controller/joints", joint_names);

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

  for (int i = 0; i < num_joints; i++)
  {
    // connect and register the joint state interface
    hardware_interface::JointStateHandle jnt_st_handle(joint_names[i], &pos[i], &vel[i], &eff[i]);
    jnt_st_interface.registerHandle(jnt_st_handle);

    // connect and register the joint velocity interface
    hardware_interface::JointHandle jnt_vel_handle(jnt_st_interface.getHandle(joint_names[i]), &vel_cmd[i]);
    jnt_vel_interface.registerHandle(jnt_vel_handle);
  }

  registerInterface(&jnt_st_interface);
  registerInterface(&jnt_vel_interface);

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
    vel[0] = plz.response.encoder1;
    vel[1] = plz.response.encoder2;
    vel[2] = plz.response.encoder3;
    vel[3] = plz.response.encoder4;
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
  insn.request.motor1 = motor_1.getCommand();
  insn.request.motor2 = motor_2.getCommand();
  insn.request.motor3 = motor_3.getCommand();
  insn.request.motor4 = motor_4.getCommand();
  insn.request.kick = true;
  insn.request.dribble = true;
  insn.request.robot = 0;  // figure out numbering scheme later
  if (cmd_client.call(insn))
  {
    return;
  }
  else
  {
    ROS_ERROR("Error communicating with robot\n");
  }
}
}
