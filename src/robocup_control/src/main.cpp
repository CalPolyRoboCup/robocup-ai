#include <robocup_control/player_hw_interface.h>
#include <robocup_control/player_control_loop.h>
#include <controller_manager/controller_manager.h>
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "player_thread");
  ros::NodeHandle n;

  ros::AsyncSpinner spinner(2);
  spinner.start();

  boost::shared_ptr<robocup_control::PlayerHWInterface> player(new robocup_control::PlayerHWInterface(n));
  player->init();
  ROS_INFO("Finished initializing player");
  robocup_control::PlayerControlLoop cl(n, player);
  ROS_INFO("Running controller");
  cl.run();
  return 0;
}
