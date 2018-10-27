/*
 * command_service.cpp
 * Copyright (C) 2018 willdle <willdle@willdle-ThinkPad-X1-Carbon>
 *
 * Distributed under terms of the MIT license.
 */

#include <robocup_control/dummy_service.h>

DummyService::DummyService(ros::NodeHandle n)
{
  command_srv = n.advertiseService("player_insns", &DummyService::respondToCommand, this);
  data_srv = n.advertiseService("player_data", &DummyService::sendData, this);
}

/*DummyService::~DummyService()
{
  delete &command_srv, &data_srv;
}*/

bool DummyService::writeToSpi(robocup_control::Insn::Request &req)
{
  return true;
}

uint8_t* formPkt(robocup_control::Insn::Request *req)
{
  uint8_t *pkt;
  pkt[0] = 0x2a;
  pkt[1] = 0xb6;
  pkt[2] = 1;
  pkt[3] = req->motor1 << 8;
  pkt[4] = req->motor1 & 0xFF;
  pkt[5] = req->motor2 << 8;
  pkt[6] = req->motor2 & 0xFF;
  pkt[7] = req->motor3 << 8;
  pkt[8] = req->motor3 & 0xFF;
  pkt[9] = req->motor4 << 8;
  pkt[10] = req->motor4 & 0xFF;
  pkt[11] = (uint8_t) req->kick;
  pkt[12] = (uint8_t) req->dribble;
  return pkt;
}

float getEncoderValue(uint8_t b1, uint8_t b2)
{
  int16_t ret;
  ret = b1 << 8;
  ret |= b2;
  return (float) ret;
}

bool DummyService::readFromSpi(robocup_control::Data::Response &resp)
{
  return true;
}

bool DummyService::respondToCommand(robocup_control::Insn::Request &req,
                      robocup_control::Insn::Response &resp)
{
  ROS_INFO("Responding to command");
  if(writeToSpi(req))
  {
    ROS_INFO("Successfully wrote stuff\n");
    ROS_INFO("Data:\n Speed 1: %d\nSpeed 2: %d\nSpeed 3: %d\nSpeed 4: %d\nKick: %d\nDribble: %d\n", req.motor1, req.motor2, req.motor3, req.motor4, req.kick, req.dribble);
    return true;
  }
  return false;
}

bool DummyService::sendData(robocup_control::Data::Request &req,
              robocup_control::Data::Response &resp)
{
  ROS_INFO("Sending data");
  if(readFromSpi(resp))
  {
    ROS_INFO("Successfully read from SPI\n");
    ROS_INFO("Data: %d\n", req.plz);
    return true;
  }
  return false;
}
