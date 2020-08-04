#include "sd_state_msgs.h"

// Running the main function for sd_state_msgs
int main(int argc, char **argv)
{
  ros::init(argc, argv, "vehicle_state_monitor");
  ROS_INFO("StreetDrone - Vehicle State Logging.");
  ros::NodeHandle nh;
  SD_STATE_MSGS sd_state_msgs(nh); //Create global object
  return 0;
}
