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
