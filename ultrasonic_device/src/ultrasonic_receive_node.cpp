#include <ros/ros.h>
#include <serial/serial.h> // http://wjwwood.io/serial/doc/1.1.0/classserial_1_1_serial.html
#include <thorvald_msgs/ThorvaldIO.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
#include <sstream>
#include <cstdint>

class UltrasonicSerial{

    public:

    // Constructor
    UltrasonicSerial(std::string port, int baud, int serial_timeout);
    ~UltrasonicSerial(){};

    // Chatter
    void chatter();

    //Variables
    int totNrDevices, deviceID, lengthInCM, count,loop;

    private:
    ros::Publisher serial_pub_;

    // Serial object
    serial::Serial serial_;

    // Ros
    ros::NodeHandle nh_;
};



UltrasonicSerial::UltrasonicSerial(std::string port, int baud, int serial_timeout):
        serial_(port, baud, serial::Timeout::simpleTimeout(serial_timeout)){


    serial_pub_ = nh_.advertise<thorvald_msgs::ThorvaldIO>("serial_io", 1);
    //serial_pub_ = nh_.advertise<std_msgs::Int16>("serial_io", 1);

    // advertise services
    std::cout << "Is the serial port open?";
    if(serial_.isOpen()){
        std::cout << " Yes." << std::endl;
        serial_.flush();}
    else{
        std::cout << " No." << std::endl;
    }

    count =0;
    loop = 0;

}



void UltrasonicSerial::chatter() {

    std::string reply = serial_.readline(100, ",\r\n");
    reply = reply.substr(0, reply.size());

    std::string NR = reply.substr(reply.find("N")+1,reply.find("I")-1);
    std::string ID = reply.substr(reply.find("I")+1,reply.find("C")-3);
    std::string CM = reply.substr(reply.find("C")+1,reply.size());


    //std::cout << reply;

    std::istringstream (NR) >> totNrDevices;
    std::istringstream (ID) >> deviceID;
    std::istringstream (CM) >> lengthInCM;


    //std::cout << "TOT: " << NR;
    //std::cout << " ID: " << ID;
    //std::cout << " CM: " << CM;

    loop++;
    std::cout << "loop: " << loop << std::endl;
    std::cout << " Tot: " << totNrDevices;
    std::cout << " ID: " << deviceID;
    std::cout << " CM: " << lengthInCM;

    if (loop>400) {

        //Publish
        thorvald_msgs::ThorvaldIO status_msg;
        status_msg.digitals.resize(2);
        //status_msg.ranges[deviceID] = lengthInCM;

        //std_msgs::Int32 status_msg;
        status_msg.digitals[0] = deviceID;
        status_msg.digitals[1] = lengthInCM;

        std::cout << deviceID << "   -   " << lengthInCM;
        count++;



        if (count == totNrDevices) {

            serial_pub_.publish(status_msg);
            count = 0;
        }
    }

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");

    std::string port(argv[1]);
    unsigned long baud = 0;
    sscanf(argv[2], "%lu", &baud);
    UltrasonicSerial msg(port, baud, 10);

    if(argc < 2) {
        ROS_WARN("Usage: uv_rig_serial <serial port address> <baudrate>");
        return 1;
    }

    //UltrasonicSerial msg;
    ros::Rate rate(30);
    //ros::Duration(3).sleep();
    while (ros::ok())
    {
    ros::spinOnce();
    msg.chatter();
    rate.sleep();
    }

    return 0;
}
