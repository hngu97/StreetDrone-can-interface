#include "sd_state_msgs.h"

SD_STATE_MSGS::SD_STATE_MSGS(ros::NodeHandle nh) : // Initialize const variable
filePath("/home/dngu0016/can-interface/databases/candatafile.dbc"),
pubRate(500), //Publish at rate 500Hz
lookupValue
  {7, 6, 5, 4, 3, 2, 1, 0,
  15, 14, 13, 12, 11, 10, 9, 8,
  23, 22, 21, 20, 19, 18, 17, 16,
  31, 30, 29, 28, 27, 26, 25, 24,
  39, 38, 37, 36, 35, 34, 33, 32,
  47, 46, 45, 44, 43, 42, 41, 40,
  55, 54, 53, 52, 51, 50, 49, 48,
  63, 62, 61, 60, 59, 58, 57, 56}
{
  ROS_INFO("sd_state_msgs.cpp: Reading CAN database with file path %s", filePath.c_str());
  int checkReadDBC = readDBC(filePath, dbcOutput);
  if (checkReadDBC == -1)
  {
    ROS_ERROR("sd_state_msgs.cpp: Invalid file path %s, please check again your file path.", filePath.c_str());
    return;
  }
  readDBCValue(filePath, dbcOutput);
  fillInCanData(dbcOutput, SDStateMsgs);
  canmsgSub = nh.subscribe("received_messages", 10, &SD_STATE_MSGS::canmsgsCallback, this); // Subscribe to CAN
  canmsgPub = nh.advertise<sd_state_msgs::SD_State_Msgs>("sd_state_msgs", 10);
  periodicPublishTimer = nh.createTimer(ros::Duration(1./pubRate), &SD_STATE_MSGS::canmsgsPublish, this); // Publishing at rate 500Hz
  ROS_INFO("sd_state_msgs.cpp: Start publising at (expected) rate %fHz.", pubRate);
  ros::spin();
}

// Destructor
SD_STATE_MSGS::~SD_STATE_MSGS()
{
}

/*----------------------------- Helper Function ----------------------------*/
// This function takes in raw message and its associated metadata, returns the decoded signal value.
void SD_STATE_MSGS::realValue(const string& bitsBig, const string& bitsLittle, const vector<Signal>& signalArray, vector<double>& resultArray)
{
  string dataBits = "";
  double result = 0;
  resultArray.clear();
  for (auto& i : signalArray)
  {
    if (i.sig_size > 32) // Check if size exceed 4 bytes
    {
      ROS_ERROR("sd_state_msgs.cpp: Currently support signal with length <= 32 bits. Your current signal length is: %d. Change to 'stoll' and 'stoull' if your signal length <= 64 bits.", i.sig_size);
      resultArray.push_back(0);
      continue;
    }

    if (i.endian) // Little End
    {
      dataBits = bitsLittle.substr(i.bit_start, i.sig_size);
      reverse(dataBits.begin(), dataBits.end());
    }
    else // Big End
    {
      dataBits = bitsBig.substr(lookupValue[i.bit_start], i.sig_size);
    }

    if (i.sig_type == 1) // IEEE754 floating point standard https://www.technical-recipes.com/2012/converting-between-binary-and-decimal-representations-of-ieee-754-floating-point-numbers-in-c/
    {
      bitset<32> float32Bits(dataBits);
      int hexNumber = float32Bits.to_ulong();
      bool negative = !!(hexNumber & 0x80000000);
      int exponent = (hexNumber & 0x7f800000) >> 23;
      int sign = negative ? -1 : 1;
      // Subtract 127 from the exponent
      exponent -= 127;
      // Convert the mantissa into decimal using the last 23 bits
      int power = -1;
      float total = 0.0;
      for (int i = 0; i < 23; i++)
      {
        int c = dataBits[i+9] - '0';
        total += (float)c * (float)pow(2.0, power);
        power--;
      }
      total += 1.0;
      result = sign * (float)pow(2.0, exponent) * total;
      result = result*i.multiplier + i.constant;
    }
    else if (i.sig_type == 0) //int
    {
      if (i.is_signed) // Convert bits to real value (int) accouting for 2s complement
      {
        long int bitsToDecimal = stol(dataBits, nullptr, 2);
        bitsToDecimal = bitsToDecimal | (bitsToDecimal &  (1UL << (i.sig_size - 1))  ? ( -(1UL << i.sig_size) ) : 0); // Dont cast to double when doing bit shift
        result = bitsToDecimal;
        result = result*i.multiplier + i.constant;
      }
      else
      {
        unsigned long int bitsToDecimal = stoul(dataBits, nullptr, 2);
        result = bitsToDecimal;
        result = result*i.multiplier + i.constant;
      }
    }
    else
    {
      result = 0;
    }
    resultArray.push_back(result);
  }
}

//Function taking in DBC_out metadata vector and clones its metadata to message_data
void SD_STATE_MSGS::fillInCanData(const vector<DataDBC>& DBC_out, sd_state_msgs::SD_State_Msgs& SDStateMsgs)
{
  for (const auto& i : DBC_out)
  {
    SDStateMsgs.can_data.message_names.push_back(i.message_name);
    SDStateMsgs.can_data.message_ids.push_back(i.message_id);
    sd_state_msgs::Message_Data message_data;
    //Fill Message Data Object
    for(const auto& j : i.signals)
    {
      message_data.signal_names.push_back(j.signal_name);
      message_data.signal_data.push_back(0);
      message_data.signal_units.push_back(j.units);
    }
    SDStateMsgs.can_data.message_data.push_back(message_data); //Push to Vector
  }
}

//Function which returns index of metadata array, given message id.
int SD_STATE_MSGS::returnIndex(const vector<DataDBC>& dbcOutput, unsigned long int msg_id)
{
  //Function which takes in msg id and returns the corresponding index of the dbcOutput vector
  //Inputs message and outputs its "index" in the DBC_out vector
  unsigned long int dbc_id = 0;
  for (int i = 0; i < dbcOutput.size(); i++)
  {
    dbc_id = dbcOutput.at(i).message_id;
    if (dbc_id == msg_id)
    {
      return i;
    }
  }
  return -1; //If message id not found
}

/*------------------------------- Callback ------------------------------*/
void SD_STATE_MSGS::canmsgsCallback(const can_msgs::Frame::ConstPtr& msg)
{
	can_msgs::Frame ReceivedFrameCAN = *msg.get();
	unsigned int frame_seq = msg->header.seq;
	int index = returnIndex(dbcOutput, msg->id); //Corresponding to index in DBC_out "metadata" vector

  //Check incase message id not found
	if (index == -1)
  {
		ROS_WARN("sd_state_msgs.cpp: Unknown Message ID: %u", msg->id);
		return;
	}

  SDStateMsgs.header = msg->header;
  //Extract message metadata from dbcOutput
  vector <Signal> signal_array = dbcOutput.at(index).signals; //array of signal objects
  string bitFormatBig = "", bitFormatLittle = "", bitString = "";
  //Calculate for both little and big endian formats
  for (int i = 0; i < msg->data.size(); i++)
  {
    bitset<8> dataBits(msg->data.at(i));
    bitString = dataBits.to_string();
    bitFormatBig = bitFormatBig + bitString;
    reverse(bitString.begin(), bitString.end());
    bitFormatLittle = bitFormatLittle + bitString;
  }
  //Calculate the signal value for a given message
  realValue(bitFormatBig, bitFormatLittle, signal_array, valueArray);
  //Decipher all signals and output to console
  SDStateMsgs.can_data.message_data.at(index).signal_data = valueArray;
}

/*------------------------------- Publishing ------------------------------*/
void SD_STATE_MSGS::canmsgsPublish(const ros::TimerEvent& event)
{
  canmsgPub.publish(SDStateMsgs);
}
