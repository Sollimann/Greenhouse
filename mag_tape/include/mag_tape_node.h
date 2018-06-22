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

class MagTapeNode
{
public:
    MagTapeNode(ros::NodeHandle &nh_);

private:

    ros::Subscriber ultrasonic_sub_;
    ros::Subscriber can_sub_;
    ros::Publisher mag_pub_;

    //constants & variables
    double Kp_x,Kp_z,Ki_x,integral_fwd;
    unsigned int totNrDevices, currentNrDevices, deviceID, range;
    bool railDetected, initialize_range_array;
    unsigned int ranges[100];

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
    void railDetection(const thorvald_msgs::ThorvaldIOConstPtr& serial_msg);

    //Adjust velocity
    void calc_velocity();


};