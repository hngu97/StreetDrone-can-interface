#include "dbc_reader.h"

/*readDBC
This function parses a dbc file path as a string and returns a variable with message metadata*/
int readDBC(const string& filePath, vector<DataDBC>& DBC_out)
{
  string output;
  DataDBC dataDBC;
  Signal signal;
  int i = 0;
  ifstream myReadFile;
  myReadFile.open(filePath.c_str());
  DBC_out.clear();
  if (myReadFile.is_open())
  {
    myReadFile >> output;
    while (!myReadFile.eof())
    {
      dataDBC.signals.clear();
      if (output.compare("BO_") != 0)
      {
        myReadFile >> output;
      }
      if (output.compare("CM_") == 0 || output.compare("BA_DEF_") == 0 || output.compare("BA_") == 0)
      {
        myReadFile >> output;
        myReadFile >> output;
        myReadFile >> output;
      }
      if (output.compare("BO_") == 0)
      {
        myReadFile >> output;
        string message_id = output;
        myReadFile >> output;
        string message_name = output;
        myReadFile >> output;
        string number_of_bytes = output;
        try // Check if the message_id is really a message_id (should be a number). If not, we know that we're in wrong streaming output, so we just continue. The same for others 'stoi'.
        {
          dataDBC.message_id = stoul(message_id);
          dataDBC.message_name = message_name;
          dataDBC.number_of_bytes = stoi(number_of_bytes);
          /*
          cout << "message name: " << dataDBC.message_name;
          cout << "\n";
          cout << "message id: " << dataDBC.message_id;
          cout << "\n";
          cout << "number of bytes: " << dataDBC.number_of_bytes;
          cout << "\n";
          cout << "signal array: ";
          cout << "\n";
	         */
          myReadFile >> output;
          myReadFile >> output; //SG_
          while (output.compare("SG_") == 0)
          {
            myReadFile >> output;
            string signal_name = output;
            signal.signal_name = signal_name;
            myReadFile >> output;
            myReadFile >> output;
            int separator = output.find("|");
            int at_sym = output.find("@");
            string bit_start = output.substr(0,separator);
            signal.bit_start = stoi(bit_start);
            string sig_size = output.substr(separator+1, at_sym-separator-1);
            signal.sig_size = stoi(sig_size);
            string endian = output.substr(at_sym+1,1);
            signal.endian = stoi(endian);
            string is_signed = output.substr(at_sym+2,1);
            signal.is_signed = (is_signed.compare("-") == 0);
            myReadFile >> output;
            string scalar = output;
            signal.scalar = scalar;
          	string scalar2 = scalar.substr(1,scalar.length()-2);
          	int separator2 = scalar2.find(",");
          	string multiplier = scalar2.substr(0,separator2);
          	string constant = scalar2.substr(separator2+1);
          	signal.multiplier = stod(multiplier);
          	signal.constant = stod(constant);
            myReadFile >> output;
            myReadFile >> output;
            string units = output;
            signal.units = units;
            dataDBC.signals.push_back(signal);
	           /*
            cout << " signal #: " << i << "\n";
            cout << " signal name: " << signal.signal_name << "\n";
            cout << " bit position start: " << signal.bit_start << "\n";
            cout << " signal size: " << signal.sig_size << "\n";
            cout << " little endian?: " << signal.endian << "\n";
            cout << " is_signed? " << signal.is_signed << "\n";
            cout << " units: " << signal.units << "\n";
            cout << " scalar: " << signal.scalar << "\n";
            cout << " multiplier: " << signal.multiplier << "\n";
            cout << " constant: " << signal.constant << "\n";
	           */
            myReadFile >> output;
            myReadFile >> output; //Must have some defense code to make this streaming logic correct
            i++;
          }
        }
        catch(...)
        {
          //ROS_INFO("dbc_reader.cpp: We're in wrong streaming. Will keep streaming...");
          continue;
        }
        // If we move to this point, it means everything is correct (hopefully). We can now push back.
        i = 0;
        DBC_out.push_back(dataDBC);
      }
    }
  }
  else
  {
    ROS_ERROR("dbc_reader.cpp: Error Reading File...");
    return -1;
  }
  ROS_INFO("dbc_reader.cpp: Finish Reading File");
  myReadFile.close();
  return 0;
}

/*readDBC
This function parses a dbc file path as a string and returns a variable with message metadata*/
void readDBCValue(const string& filePath, vector<DataDBC>& DBC_out)
{
  string output;
  int i = 0;
  ifstream myReadFile;
  myReadFile.open(filePath.c_str());
  if (myReadFile.is_open())
  {
    myReadFile >> output;
    while (!myReadFile.eof()) //As long as we havent reached the end
    {
      if (output.compare("SIG_VALTYPE_") != 0) //If its not SIG_VALTYPE_, move ahead
      {
        myReadFile >> output;
      }
      else //If it is SIGVALUE TYPE, then:
      {
	      try
        {
	        //Extract information in row
          myReadFile >> output;//Message ID
	        string m = output;
	        unsigned long MsgId = stoul(m);
	        //cout << "Extracted Message ID: "<< MsgId << endl;
          myReadFile >> output; //Signal Name
	        //cout << "Signal Name: " << output << endl;
	        string SigName = output;
          myReadFile >> output;
          myReadFile >> output; //Signal Value Type
	        string t = output.substr(0,1);
	        int SigType = stoi(t); //<- Seems to cause problems when uncommented out
	        //cout << "Signal Val Type: "<< SigType << endl;
          //Fill into dbcOutput vector
	        int MsgIndex = returnMsgIndex(DBC_out, stoi(m));
	        int SigIndex = returnSignalIndex(DBC_out,SigName,MsgIndex);
	        DBC_out.at(MsgIndex).signals.at(SigIndex).sig_type = stoi(t);
          i++;
       	}
	      catch(...)
        {
          continue;
        }
      }
      i = 0;
    }
  }
  else
  {
    ROS_ERROR("dbc_reader.cpp: Error Reading File...");
  }
  myReadFile.close();
  /*for(int i = 0; i < DBC_out.size(); i++)
  {
    cout << "Message Id: " << DBC_out.at(i).message_id << " -------" << endl;
    int signo = DBC_out.at(i).signals.size();
  	for(int j = 0; j < DBC_out.at(i).signals.size(); j++)
    {
  	  cout << "Signal Name: " << DBC_out.at(i).signals.at(j).signal_name << ". " << "Signal Type: " << DBC_out.at(i).signals.at(j).sig_type << endl;
  	}
  }*/
}

//Function which takes in msg id and returns the corresponding index of the dbcOutput vector
//Inputs message and outputs its "index" in the DBC_out vector
int returnMsgIndex(const vector<DataDBC>& dbcOutput, unsigned long int MsgId)
{
  for (int i = 0; i < dbcOutput.size(); i++)
  {
  	unsigned long int dbc_id = dbcOutput.at(i).message_id;
  	if (MsgId == dbc_id)
    {
  		return i;
  	}
  }
  return -1; //If message id not found
}


//New function here which takes in signal name string and returns the int index on dbcOutput[index].signals[...]
int returnSignalIndex(const vector<DataDBC>& DBC_out, const string& signalName, int index)
{
  for(int i = 0; i < DBC_out.at(index).signals.size(); i++)
  {
		string name = DBC_out.at(index).signals.at(i).signal_name;
		if(signalName == name)
    {
      return i;
		}
  }
	return -1; //No signal found for given message id.
}
