# CAN Interface

## Purpose
A ros node that subscribes to "/received_messages" and publishes a topic for each CAN message in /sd_state_msgs.

## To Use
### Data Output Summary
Data is published as struct 'can_data' with corresponding vectors 'message_names', 'message_ids', and 'message_data'. 'message_data' contains two arrays, 'signal_data', 'signal_names', and 'signal_units'.
- `cd can-interface/catkin_ws`
- `source devel/setup.bash`
- `rosmsg show sd_state_msgs/SD_State_Msgs`

        std_msgs/Header header
                uint32 seq
                time stamp
                string frame_id
        sd_state_msgs/Can_Data can_data
                string[] message_names
                uint16[] message_ids
                sd_state_msgs/Message_Data[] message_data
                    string[] signal_names
                    float64[] signal_data
                    string[] signal_units

### Examples
#### To get index corresponding to the wheel speed message and their data (id 280)

    #include "ros/ros.h"
    #include <sd_state_msgs/SD_State_Msgs.h>
    #include <iostream>

    void dataCallback(const sd_state_msgs::SD_State_Msgs::ConstPtr& msg)
    {
        unsigned int wheel_speeds_index = 0;
        for(int i = 0; i < msg->can_data.message_ids.size(); i++)
        {
            if (msg->can_data.message_ids[i] == 280)
            {
                wheel_speeds_index = i;
                break;
            }   
        }

        std::string msgName = msg->can_data.message_names[wheel_speeds_index];
        unsigned int msgId = msg->can_data.message_ids[wheel_speeds_index];
        std::vector<std::string> sigNames = msg->can_data.message_data[wheel_speeds_index].signal_names;
        std::vector<double> sigData = msg->can_data.message_data[wheel_speeds_index].signal_data;
        std::vector<std::string> sigUnits = msg->can_data.message_data[wheel_speeds_index].signal_units;

        std::cout << msgName << ", " << msgId << std::endl;
        for (int i = 0; i < sigData.size(); i++)
        {
            std::cout << (i+1) << ":" << sigNames[i] << ", " << sigData[i] << ", " << sigUnits[i] << std::endl;
        }
        std::cout << "--------------------------------------------" << std::endl;
    }

    int main(int argc, char **argv)
    {
        ros::init(argc, argv, "mytest");
        ros::NodeHandle nh;
        ros::Subscriber sub = nh.subscribe("sd_state_msgs", 10, dataCallback);
        ros::spin();
        return 0;
    }

#### CMakeLists.txt
    #...

    find_package(catkin REQUIRED COMPONENTS
    #...
    sd_state_msgs
    )

    #...

    add_executable(your_cpp_node src/your_cpp_node.cpp)
    target_link_libraries(your_cpp_node ${catkin_LIBRARIES})
    add_dependencies(your_cpp_node ${catkin_EXPORTED_TARGETS})

#### package.xml
    <!-- ... -->

    <build_depend>sd_state_msgs</build_depend>
    <exec_depend>sd_state_msgs</exec_depend>

    <!-- ... -->
    
### Message Index Lookup File (For commonly used CAN messages)
See 'message_index.pdf'

# Dev Stuff
## Learning
The details of each message can be found inside the candatafile.dbc

For more info on dbc files look here: https://medium.com/@energee/what-are-dbc-files-469a3bf9b04b

For big or little frame layout: https://doc.micrium.com/display/candoc/Frame+Layout+-+Little+or+Big+Endian

## To Run (kinetic)
#### Terminal Tab 1 (if simulating)
- `cd can-interface/bags`
- `rosbag play -l 2020-02-28-12-19-00.bag` 
#### Terminal Tab 2
- `cd can-interface/catkin_ws`
- `catkin_make`
- `source devel/setup.bash`
- `roslaunch sd_state_msgs sd_state_msgs.launch` 
#### Terminal Tab 4
- `cd can-interface/catkin_ws`
- `source devel/setup.bash`
- `rostopic echo -n1 /sd_state_msgs`

## If an error occurs:
- Delete devel and build
- `cd can-interface/catkin_ws`
- `rosdep install --from-paths src --ignore-src`
- `catkin_make`
- `source devel/setup.bash`

