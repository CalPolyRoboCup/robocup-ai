/*
 * command_thread.cpp
 * Copyright (C) 2018 willdle <willdle@willdle-ThinkPad-X1-Carbon>
 *
 * Distributed under terms of the MIT license.
 */

//#include <robocup_control/DummyService.h>
//#include <robocup_control/PlayerHWInterface.h>
#include <robocup_control/player_hw_interface.h>
#include <robocup_control/sim_spi_service.h>
#include <ros/ros.h>
using namespace std;
int main(int argc, char** argv)
{
  ros::init(argc, argv, "sim_spi_server");
  ros::NodeHandle n;
  cout << "Starting Sim SPI service\n" << endl;
  SimSpiService srv = SimSpiService(n);
  while (ros::ok())
  {
    ros::spin();
  }
  return 0;
}
