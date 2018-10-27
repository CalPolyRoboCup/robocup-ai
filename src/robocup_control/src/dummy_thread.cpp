/*
 * command_thread.cpp
 * Copyright (C) 2018 willdle <willdle@willdle-ThinkPad-X1-Carbon>
 *
 * Distributed under terms of the MIT license.
 */

//#include <robocup_control/DummyService.h>
//#include <robocup_control/PlayerHWInterface.h>
#include <robocup_control/player_hw_interface.h>
#include <robocup_control/dummy_service.h>
#include <ros/ros.h>
using namespace std;
int main(int argc, char** argv)
{
  ros::init(argc, argv, "dummy_server");
  ros::NodeHandle n;
  cout << "Starting dummy service\n" << endl;
  DummyService srv = DummyService(n);
  while(ros::ok())
  {
    ros::spin();
  }
  return 0;
}
