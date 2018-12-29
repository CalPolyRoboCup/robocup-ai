/*
 * dummy_pose_pub.cpp
 * Copyright (C) 2018 willdle <willdle@willdle-ThinkPad-X1-Carbon>
 *
 * Distributed under terms of the MIT license.
 */

#include <geometry_msgs/Pose.h>
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dummy_pose");
  ros::NodeHandle n;
  ros::Publisher pose_pub = n.advertise<geometry_msgs::Pose>("command_pose", 1000);
  ros::Rate loop_rate(60);
  while(ros::ok())
  {
    geometry_msgs::Pose pose;
    pose.position.x = 0.0;
    pose.position.y = 1.0;
    pose.position.z = 0.0;
    ROS_INFO("Sending new pose: x = %f, y = %f, theta= %f", pose.position.x, pose.position.y, pose.position.z);
    pose_pub.publish(pose);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
