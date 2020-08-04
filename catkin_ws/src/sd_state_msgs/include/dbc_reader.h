#ifndef DBC_READER_H
#define DBC_READER_H

#include <vector>
#include <string.h>
#include <string>
#include <iostream>
#include <fstream>
#include "ros/ros.h"

using namespace std;

struct Signal
{
  string signal_name;
  int bit_start;
  int sig_size;
  bool endian; // 1:little or 0:big
  bool is_signed;
  string units;
  string scalar;
  double multiplier;
  double constant;
  int sig_type = 0; //For signal type. 0 for int, 1 for double.
};

struct DataDBC
{
  string message_name;
  unsigned int message_id; //There is one ID that exceeds 'int' size
  int number_of_bytes;
  vector<Signal> signals;
};


int returnSignalIndex(const vector<DataDBC>& DBC_out, const string& signalName, int index);
int returnMsgIndex(const vector<DataDBC>& dbcOutput, unsigned long int MsgId);
int readDBC(const string& filePath, vector<DataDBC> &DBC_out);
void readDBCValue(const string& filePath, vector<DataDBC>& DBC_out);


#endif /* end of include guard:  */
