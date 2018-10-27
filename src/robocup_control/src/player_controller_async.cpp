/*
 * player_controller_async.cpp
 * Copyright (C) 2018 willdle <willdle@willdle-ThinkPad-X1-Carbon>
 *
 * Distributed under terms of the MIT license.
 */

#include <robocup_control/player_hw_interface.h>
#include <controller_manager/controller_manager.h>
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "player_thread");
  ros::NodeHandle n;
  PlayerHWInterface player = PlayerHWInterface(n);
  controller_manager::ControllerManager cm(&player);
  ros::Rate loop_rate(60);
  double last_time = ros::Time::now().toSec();
  double duration = 0.0;
  while (ros::ok())
  {
    ROS_INFO("The player thread is running");
    duration = last_time - ros::Time::now().toSec();
    ros::Duration d(duration);
    player.read();
    cm.update(ros::Time::now(), d);
    player.write();
    loop_rate.sleep();
    last_time = ros::Time::now().toSec();
  }
  return 0;
}
