/*
 * command_service.cpp
 * Copyright (C) 2018 willdle <willdle@willdle-ThinkPad-X1-Carbon>
 *
 * Distributed under terms of the MIT license.
 * #include <random>
 * #include <robocup_control/Insn.h>
 * #include <robocup_control/Data.h>
 * #include <ros/ros.h>
 * #include <sys/types.h>
 * #include <sys/socket.h>
 * #include <robocup_control/grSim_Packet.pb.h>
 */

#include <robocup_control/dummy_service.h>

DummyService::DummyService(ros::NodeHandle n)
{
  command_srv = n.advertiseService("player_insns", &DummyService::respondToCommand, this);
  data_srv = n.advertiseService("player_data", &DummyService::sendData, this);
  //last_vel.resize(4);
  curr_vel.resize(4);
  //create socket
  sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if ( sock < 0 )
  {
    ROS_ERROR("Failed to make socket on line: %d in %s", __LINE__, __FILE__);
  }
  //form sock_addr struct
  addr.sin_family = AF_INET;
  addr.sin_port = htons(20011);
  addr.sin_addr.s_addr = inet_addr("127.0.0.1");
}

DummyService::~DummyService()
{
  //sock.close();
  //delete sock;
}

bool DummyService::writeToSpi(robocup_control::Insn::Request &req)
{
  grSim_Packet pkt;
  grSim_Robot_Command* cmd;
  char* msg;
  int err;
  pkt.mutable_commands()->set_isteamyellow(true);
  pkt.mutable_commands()->set_timestamp(0.0);
  cmd = pkt.mutable_commands()->add_robot_commands();
  curr_vel[0] = req.motor1;
  curr_vel[1] = req.motor2;
  curr_vel[2] = req.motor3;
  curr_vel[3] = req.motor4;
  //form grSim pkt
  cmd->set_id(req.robot);
  cmd->set_wheelsspeed(true);
  cmd->set_wheel1(req.motor1);
  cmd->set_wheel2(req.motor2);
  cmd->set_wheel3(req.motor3);
  cmd->set_wheel4(req.motor4);
  cmd->set_veltangent(0.0);
  cmd->set_velnormal(0.0);
  cmd->set_velangular(0.0);
  if (req.kick)
  {
    cmd->set_kickspeedx(6.5);
  }
  else 
  {
    cmd->set_kickspeedx(0.0);
  }
  cmd->set_kickspeedz(0.0);
  cmd->set_spinner(req.dribble);
  size_t size = pkt.ByteSize();
  msg = (char *) malloc(size);
  if (msg < 0)
  {
    ROS_ERROR("Bad Malloc: %d in %s", __LINE__, __FILE__);
  }

  pkt.SerializeToArray(msg, size);
  //send grSim pkt
  err = sendto(sock, msg, size, 0, (struct sockaddr* ) &addr, sizeof(addr));
  if ( err < 0 )
  {
    ROS_ERROR("Bad Send: %d in %s", __LINE__, __FILE__);
  }
  return true;
}

bool DummyService::readFromSpi(robocup_control::Data::Response &resp)
{
  static double means[4] = {0, 0, 0, 0};
  double mean;
  
  for(int i = 0; i < curr_vel.size(); i++)
  {
    mean = curr_vel[i];
    std::normal_distribution<double> distribution(mean, 2.0);
    mean = distribution(generator);
    means[i] = mean;
  }
  ROS_INFO("Encoder readings: %lf, %lf, %lf, %lf", 
      means[0], means[1], means[2], means[3]);
  resp.encoder1 = means[0];
  resp.encoder2 = means[1];
  resp.encoder3 = means[2];
  resp.encoder4 = means[3];
  resp.kicker_ready = false;
  resp.dribble_plz = false;
  //last_vel = curr_vel;
  return true;
}

bool DummyService::respondToCommand(robocup_control::Insn::Request &req,
                      robocup_control::Insn::Response &resp)
{
  //ROS_INFO("Responding to command");
  if(writeToSpi(req))
  {
    //ROS_INFO("Successfully wrote stuff\n");
    //ROS_INFO("Data:\n Speed 1: %lf\nSpeed 2: %lf\nSpeed 3: %lf\nSpeed 4: %lf\nKick: %lf\nDribble: %lf\n",
        //req.motor1, req.motor2, req.motor3, req.motor4, req.kick, req.dribble);
    return true;
  }
  return false;
}

bool DummyService::sendData(robocup_control::Data::Request &req,
              robocup_control::Data::Response &resp)
{
  //ROS_INFO("Sending data");
  if(readFromSpi(resp))
  {
    //ROS_INFO("Successfully read from SPI\n");
    //ROS_INFO("Data: %d\n", req.plz);
    return true;
  }
  return false;
}
