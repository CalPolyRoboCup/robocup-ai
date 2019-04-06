#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dummy_speed");
  ros::NodeHandle n;
  ros::Publisher speed_pub = n.advertise<geometry_msgs::Twist>("command_speed", 1000);
  ros::Rate loop_rate(60);
  while (ros::ok())
  {
    geometry_msgs::Twist twist;
    twist.linear.x = 0.0;
    twist.linear.y = 1.0;
    twist.angular.z = 0.0;
    ROS_INFO("Sending new twist: x = %f, y = %f, theta= %f", twist.linear.x, twist.linear.y, twist.angular.z);
    speed_pub.publish(twist);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
