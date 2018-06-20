#include <ros/ros.h>
#include <serial/serial.h> // http://wjwwood.io/serial/doc/1.1.0/classserial_1_1_serial.html
#include <thorvald_msgs/ThorvaldIO.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <sstream>

class UltrasonicSerial{

    public:

    // Constructor
    UltrasonicSerial(std::string port, int baud, int serial_timeout);
    UltrasonicSerial();

    // Chatter
    void chatter();

    private:
    ros::Publisher serial_pub_;

    // Serial object
    serial::Serial serial_;

    // Ros
    ros::NodeHandle nh_;

};

/*
UltrasonicSerial::UltrasonicSerial() {

    serial_pub_ = nh_.advertise<std_msgs::String>("chatter",1);
}
*/



UltrasonicSerial::UltrasonicSerial(std::string port, int baud, int serial_timeout):
        serial_(port, baud, serial::Timeout::simpleTimeout(serial_timeout)){


    serial_pub_ = nh_.advertise<std_msgs::String>("chatter",1);

    // advertise services
    std::cout << "Is the serial port open?";
    if(serial_.isOpen())
        std::cout << " Yes." << std::endl;
    else
        std::cout << " No." << std::endl;

}


void UltrasonicSerial::chatter() {

    std_msgs::String string;
    std::stringstream ss;
    ss << "hello world ";
    string.data = ss.str();
    serial_pub_.publish(string);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");


    std::string port(argv[1]);
    unsigned long baud = 0;
    sscanf(argv[2], "%lu", &baud);
    UltrasonicSerial msg(port, baud, 1000);

    if(argc < 2) {
        ROS_WARN("Usage: uv_rig_serial <serial port address> <baudrate>");
        return 1;
    }



    //UltrasonicSerial msg;
    ros::Rate rate(20);
    while (ros::ok())
    {
    ros::spinOnce();
    msg.chatter();
    rate.sleep();
    }

    return 0;
}
