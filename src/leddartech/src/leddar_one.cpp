#include "ros/ros.h"
#include "ros/console.h"
#include "std_msgs/String.h"
#include "leddartech/leddar_data.h"

#include <sstream>
#include <string>

#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>

#include "Leddar.h"

using namespace std;

class LeddarOne {
private:
    ros::Publisher publisher;
    ros::NodeHandle publisherNodeHandle;

    const string publishTopic;

    string portName;
    int modbusAddress;
    LtAcquisition ltAcquisition;

public:
    LeddarOne(ros::NodeHandle& publisherNodeHandle,
              string publishTopic = "",
              string portName = "ttyUSB2",
              int modbusAddress = 1) :
        publisherNodeHandle(publisherNodeHandle),
        publishTopic(publishTopic),
        portName(portName),
        modbusAddress(modbusAddress) {
        publisher = publisherNodeHandle.advertise<leddartech::leddar_data>(
                    publishTopic, 1000);
    }

    bool connect() {
        char* cStringPortName = new char[LT_MAX_PORT_NAME_LEN+1];
        strcpy(cStringPortName, portName.c_str());

        bool connectionSuccess =
                (LeddarConnect(cStringPortName , modbusAddress) == LT_SUCCESS);

        delete cStringPortName;

        return connectionSuccess;
    }

    bool publishData() {
        leddartech::leddar_data leddarData;

        if ( LeddarGetResults( &ltAcquisition ) == LT_SUCCESS )
        {
            LtDetection *lDetections = ltAcquisition.mDetections;

            leddarData.timestamp = ltAcquisition.mTimestamp;
            leddarData.temperature = ltAcquisition.mTemperature;
            leddarData.distance = lDetections[0].mDistance;
            leddarData.amplitude = lDetections[0].mAmplitude;
            //            for(int i = 0; i < ltAcquisition.mDetectionCount; ++i)
            //            {
            //            }
        }
        else
        {
            ROS_ERROR_STREAM("\nCommunication error, aborting.\n");
            return false;
        }

        publisher.publish(leddarData);

        return true;
    }

    void disconnect() {
        LeddarDisconnect();
    }

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "leddar_one");
    ros::NodeHandle nodeHandle("~");
    string publishTopic;
    string portName;
    int modbusAddress;
    nodeHandle.param<string>("publishTopic", publishTopic, "");
    nodeHandle.param<string>("portName", portName, "ttyUSB0");
    nodeHandle.param<int>("modbusAddress", modbusAddress, 1);

    ROS_INFO("===== Leddar one node started =====");
    ROS_INFO_STREAM("publishTopic: " << publishTopic);
    ROS_INFO_STREAM("portName: " << portName);
    ROS_INFO_STREAM("modbusAddress: " << modbusAddress);

    LeddarOne leddarOne(nodeHandle, publishTopic, portName, modbusAddress);

    if (leddarOne.connect() == false) {
        ROS_ERROR_STREAM("Leddar One connection failed!");
        ROS_ERROR_STREAM("Make sure permission is granted on ttyUSB0:");
        ROS_ERROR_STREAM("sudo chown :your_user_name /dev/ttyUSB0");
        ROS_ERROR_STREAM("Make sure -> Port name: ttyUSB0, Modbus address: 1");

        return 0;
    }

    while(ros::ok()){
        leddarOne.publishData();
        ros::spinOnce();
    }
    leddarOne.disconnect();

    return 0;
}
