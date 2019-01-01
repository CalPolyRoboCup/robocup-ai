/*
 * player_control_loop.h
 * Copyright (C) 2018 willdle <willdle@willdle-ThinkPad-X1-Carbon>
 *
 * Distributed under terms of the MIT license.
 */

#ifndef PLAYER_CONTROL_LOOP_H
#define PLAYER_CONTROL_LOOP_H

#include <time.h>
#include <boost/shared_ptr.hpp>
#include <robocup_control/player_hw_interface.h>

namespace robocup_control
{
static const double BILLION = 1000000000.0;
/**
 * @brief Class for running the controller for RoboCup robots
 */
class PlayerControlLoop
{
public:
  /**
   * @brief Constructor
   * @param nh - Node handle for topics and services
   * @param hw_interface - Interface for RoboCup robot hardware
   */
  PlayerControlLoop(ros::NodeHandle& nh, boost::shared_ptr<robocup_control::PlayerHWInterface> hw_interface);
  /**
   * @brief Run controller
   */
  void run();

protected:
  /**
   * @brief update function
   */
  void update();

  // internal node handle for roscpp prgm
  ros::NodeHandle nh;
  // class name

  // timing settings
  ros::Duration update_period;
  double cycle_time_error_thresh;

  // timing variables
  ros::Duration elapsed_time;
  double loop_hz;
  struct timespec last_time;
  struct timespec curr_time;

  // serializes and runs the controller
  boost::shared_ptr<controller_manager::ControllerManager> ctrl_manager;

  // hardware interface for RoboCup robots
  boost::shared_ptr<robocup_control::PlayerHWInterface> hw_interface;
};
}

#endif /* !PLAYER_CONTROL_LOOP_H */
