#ifndef SD_STATE_MSGS_H
#define SD_STATE_MSGS_H

#include "ros/ros.h"
#include <can_msgs/Frame.h>
#include <iostream>
#include <bitset>
#include <algorithm>
#include <math.h>
#include <limits.h>
#include "dbc_reader.h"
#include <sd_state_msgs/SD_State_Msgs.h>

class SD_STATE_MSGS
{
public:
  // Constructor
  SD_STATE_MSGS(ros::NodeHandle nh);
  // Destructor
  ~SD_STATE_MSGS();

private:
  // Variable Declaration
  const string filePath;
  const int lookupValue[64];
  const float pubRate;
  vector<double> valueArray;
  vector<DataDBC> dbcOutput;
  ros::Subscriber canmsgSub;
  ros::Publisher canmsgPub;
  ros::Timer periodicPublishTimer;
  sd_state_msgs::SD_State_Msgs SDStateMsgs;

  /*----------------------------- Helper Function ----------------------------*/
  // This function takes in raw message and its associated metadata, returns the decoded signal value.
  void realValue(const string& bitsBig, const string& bitsLittle, const vector<Signal>& signalArray, vector<double>& resultArray);
  //Function taking in DBC_out metadata vector and clones its metadata to message_data
  void fillInCanData(const vector<DataDBC>& DBC_out, sd_state_msgs::SD_State_Msgs& SDStateMsgs);
  //Function which returns index of metadata array, given message id.
  int returnIndex(const vector<DataDBC>& dbcOutput, unsigned long int msg_id);



  /*------------------------------- Callback ------------------------------*/
  void canmsgsCallback(const can_msgs::Frame::ConstPtr& msg);

  /*------------------------------- Publishing ------------------------------*/
  void canmsgsPublish(const ros::TimerEvent& event);
};

#endif
