#include <ros/ros.h>
#include <serial/serial.h> // http://wjwwood.io/serial/doc/1.1.0/classserial_1_1_serial.html
#include <thorvald_msgs/ThorvaldIO.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
#include <sstream>
#include <cstdint>

/****************************************************************************/
/**************************** Class declaration *****************************/
/****************************************************************************/

class UltrasonicSerial{

    public:

    // Constructor
    UltrasonicSerial(std::string port, int baud, int serial_timeout);
    ~UltrasonicSerial(){};

    // Chatter
    void chatter();

    //Variables
    int totNrDevices, deviceID, lengthInCM;

    private:
    ros::Publisher serial_pub_;
    thorvald_msgs::ThorvaldIO status_msg;

    // Serial object
    serial::Serial serial_;

    // Ros
    ros::NodeHandle nh_;
};
