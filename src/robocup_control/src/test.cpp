/*
 * test.cpp
 * Copyright (C) 2018 willdle <willdle@willdle-ThinkPad-X1-Carbon>
 *
 * Distributed under terms of the MIT license.
 */

#include "ros/ros.h"
#include <robocup_control/Insn.h>
#include <robocup_control/Data.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_thread");
    
    ros::NodeHandle nh;
    ros::ServiceClient cmd_client = nh.serviceClient<robocup_control::Insn>("player_insns");
    ros::ServiceClient data_client = nh.serviceClient<robocup_control::Data>("player_data");
    
    robocup_control::Data plz;
    plz.request.plz = true;
    
    if (data_client.call(plz))
    {
      ROS_INFO("Receiving data");
      /*
       * motor1.setVelocity();
       * motor2.setVelocity();
       * motor3.setVelocity();
       * motor4.setVelocity();
       */
    }
    else
    {
      ROS_ERROR("Error communicating with robot\n");
    }

   robocup_control::Insn insn;
   insn.request.motor1 = 0;
   insn.request.motor2 = 1;
   insn.request.motor3 = 2;
   insn.request.motor4 = 3;
   insn.request.kick = true;
   insn.request.dribble = true;
   
   if (cmd_client.call(insn))
   {
     ROS_INFO("Sending commands");
   }
   else
   {
     ROS_ERROR("Error communicating with robot\n");
   } 
   return 0;
}
