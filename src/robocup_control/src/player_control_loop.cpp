/*
 * player_control_loop.cpp
 * Copyright (C) 2018 willdle <willdle@willdle-ThinkPad-X1-Carbon>
 *
 * Distributed under terms of the MIT license.
 */

#include <robocup_control/player_control_loop.h>

namespace robocup_control
{

PlayerControlLoop::PlayerControlLoop(ros::NodeHandle& nh, boost::shared_ptr<robocup_control::PlayerHWInterface> hw_interface) : nh(nh), hw_interface(hw_interface)
{
  ROS_INFO("Starting to intialize control loop");
  ctrl_manager.reset(new controller_manager::ControllerManager(hw_interface.get(), nh));
  ROS_INFO("Started controller manager");
  //load loop_rate
  loop_hz = 60;
  //load error threshold
  cycle_time_error_thresh = 10;

  update_period = ros::Duration(1/ loop_hz);
  clock_gettime(CLOCK_MONOTONIC, &last_time);
  ROS_INFO("Done creating contorl loop");
}

void PlayerControlLoop::run()
{
  ros::Rate rate(loop_hz);
  while(ros::ok())
  {
    //ROS_INFO("Running control loop");
    update();
    rate.sleep();
  }
}

void PlayerControlLoop::update()
{
  clock_gettime(CLOCK_MONOTONIC, &curr_time);
  elapsed_time = ros::Duration(curr_time.tv_sec - last_time.tv_sec + (curr_time.tv_nsec - last_time.tv_nsec) / 1000000000);
  last_time = curr_time;
  const double error = (elapsed_time - update_period).toSec();
  if (error > cycle_time_error_thresh)
  {
      ROS_WARN_STREAM_NAMED("player_control_loop", "Cycle time exceeded error threshold by: "<< error << ", cycle time: " << elapsed_time << ", threshold: " << cycle_time_error_thresh);
  }
  //ROS_INFO("Trying to read values");
  hw_interface->read();
  //ROS_INFO("Read values");
  //ROS_INFO("Updating control manager");
  ctrl_manager->update(ros::Time::now(), elapsed_time);
  //ROS_INFO("Updated control manager");
  //ROS_INFO("Writing to robot");
  hw_interface->write();
  //ROS_INFO("Done writing values");

}
        
}
