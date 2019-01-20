#ifndef SIM_SPI_SERVICE_H
#define SIM_SPI_SERVICE_H
#include <random>
#include <robocup_control/Insn.h>
#include <robocup_control/Data.h>
#include <ros/ros.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include "grSim_Packet.pb.h"

/**
 * @brief Class for simulating the SPI communication between the RPi and STM32
 */
class SimSpiService
{
private:
  ros::NodeHandle n;
  ros::ServiceServer command_srv;
  ros::ServiceServer data_srv;
  std::vector<double> curr_vel;
  std::default_random_engine generator;
  int sock;
  struct sockaddr_in addr;

  /**
   * @brief sends SPI commands to STM board to actuators
   * @param req a command from the main CPU
   * @return true if SPI write succeeds
   */
  bool writeToSpi(robocup_control::Insn::Request& req);

  /**
   * @brief reads SPI commands from STM board to prepare to send data back to main CPU
   * @param resp a struct Data::Respond to send to the main CPU
   * @return true if SPI read succeeds
   */
  bool readFromSpi(robocup_control::Data::Response& resp);

public:
  /**
   * @brief Creates new CommandService that receives commands from main CPU,
   * also, sends data back to main CPU
   * @param n NodeHandle for creating services
   */
  SimSpiService(ros::NodeHandle n);
  ~SimSpiService();

  /**
   * @brief callback that handles command from main CPU
   * @param req Command from main CPU
   * @param resp true if request was correctly received and processed
   **/
  bool respondToCommand(robocup_control::Insn::Request& req, robocup_control::Insn::Response& resp);

  /**
   * @brief callback that handles a request for data from main CPU
   * @param req Request for data from main CPU
   * @param resp Data from robot to send back to main CPU
   **/
  bool sendData(robocup_control::Data::Request& req, robocup_control::Data::Response& resp);
};

#endif /* !SIM_SPI_SERVICE_H */
