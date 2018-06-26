#include <ros/ros.h>
#include <thorvald_base/CANFrame.h>
#include <thorvald_msgs/ThorvaldIO.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>

#define dt 0.05

int SENSOR_LEFT_LIMIT = -100;
int SENSOR_RIGHT_LIMIT = 100;
int SENSOR_CENTER_VALUE = 0;
int SENSOR_MAX_ERROR = 100;



class RailWay{

public:

    //railWay class can access MagTape class
    friend class MagTape;
    //Constructor
    bool automaticPlantMonitoring();
    //friend void RailWay::automaticPlantMonitoring();
    bool enterRail, leaveRail;

    bool railDetection();

private:

    bool railDetected, tapeDetected;
    double railLength;
    unsigned int totNrDevices, currentNrDevices, deviceID, range;
    unsigned int ranges[100];

    RailWay(){};
    //Desctructor
    //~RailWay();


};

class MagTape : private RailWay
{
public:
    MagTape(ros::NodeHandle &nh_);
    MagTape(){};

    //MagTape can access RailWay
    //friend class RailWay;


    // Object
    //RailWay rails;
    friend class RailWay;
    //bool railDetected, tapeDetected;
private:

    ros::Subscriber ultrasonic_sub_;
    ros::Subscriber can_sub_;
    ros::Publisher mag_pub_;



    //constants & variables
    double Kp_x,Kp_z,Ki_x,integral_fwd, error;

    //friend RailWay bool railDetected, tapeDetected
    bool initialize_range_array, railMonitoringMission;


    //Initialize
    geometry_msgs::Twist base_cmd;

    //Magnetic tape marker
    //Ascii
    signed char left_marker;
    signed char right_marker;

    //can messages callback
    void canCallback(const thorvald_base::CANFrameConstPtr& can_msg);

    //Check if tape is nearby
    bool tapeDetection();

    //Check if connected to rail
    void rangeDetection(const thorvald_msgs::ThorvaldIOConstPtr& serial_msg);

    //Adjust velocity along tape
    void calc_velocity_along_tape();

    //Adjust velocity along rail
    //void calc_velocity_along_rail();

    //Monitor plants along rail

    //Control vechicle
    void controller();
};


