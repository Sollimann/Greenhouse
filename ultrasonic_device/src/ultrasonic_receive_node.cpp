#include "ultrasonic_receive_node.h"

/****************************************************************************/
/********************************* CONSTRUCTOR ******************************/
/****************************************************************************/

UltrasonicSerial::UltrasonicSerial(std::string port, int baud, int serial_timeout):
        serial_(port, baud, serial::Timeout::simpleTimeout(serial_timeout)){


    serial_pub_ = nh_.advertise<thorvald_msgs::ThorvaldIO>("serial_io", 1);

    // advertise services
    std::cout << "Is the serial port open?";
    if(serial_.isOpen()){
        std::cout << " Yes." << std::endl;
        serial_.flush();}
    else{
        std::cout << " No." << std::endl;
    }


}


/****************************************************************************/
/*********************************  Publisher  ******************************/
/****************************************************************************/


void UltrasonicSerial::chatter() {

    std::string reply = serial_.readline(100, ",\r\n");
    reply = reply.substr(0, reply.size());

    std::string NR = reply.substr(reply.find("N")+1,reply.find("I")-1);
    std::string ID = reply.substr(reply.find("I")+1,reply.find("C")-3);
    std::string CM = reply.substr(reply.find("C")+1,reply.size());

    std::istringstream (NR) >> totNrDevices;
    std::istringstream (ID) >> deviceID;
    std::istringstream (CM) >> lengthInCM;

    std::cout << " Tot: " << totNrDevices;
    std::cout << " ID: " << deviceID;
    std::cout << " CM: " << lengthInCM << std::endl;

    //Publish
    status_msg.ranges.resize(totNrDevices);
    status_msg.analogs.resize(2);

    status_msg.analogs[0] = deviceID;	
    status_msg.analogs[1] = totNrDevices;
    status_msg.ranges[deviceID] = lengthInCM;


    serial_pub_.publish(status_msg);
}

/****************************************************************************/
/******************************** ROS SPECIFIC ******************************/
/****************************************************************************/

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
    ros::Rate rate(40);
    while (ros::ok())
    {
    ros::spinOnce();
    msg.chatter();
    rate.sleep();
    }

    return 0;
}
