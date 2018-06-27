// Ros
#include <ros/ros.h>

// Messages
#include <thorvald_msgs/ThorvaldIO.h>
#include <geometry_msgs/Twist.h>

// Services
#include <thorvald_base/CANFrame.h>
#include <std_srvs/Trigger.h>
#include <semi_auto/PlantMonitoring.h>
#include <semi_auto/AddTwoInts.h>


class SemiAuto
{
public:
    SemiAuto(ros::NodeHandle &nh_);

    // Functions
    bool railDetection();
    bool wallDetection();
    void automaticRailService();

private:

    /** Service **/

    bool add(semi_auto::AddTwoInts::Request  &req,
             semi_auto::AddTwoInts::Response &res);

    bool plant_monitoring(std_srvs::Trigger::Request  &req,
             std_srvs::Trigger::Response &res);

    // Service declaration
    ros::ServiceServer add_ints_service_;
    ros::ServiceServer plant_monitoring_service_;


    /** Subscriber **/

    ros::Subscriber ultrasonic_sub_;

    /** Publisher **/

    ros::Publisher vel_pub_;


    /** Callbacks **/

    //Check if connected to rail
    void rangeDetection(const thorvald_msgs::ThorvaldIOConstPtr& serial_msg);

    /** Variables **/
    bool railDetected, wallDetected, forwardMotion, plantServiceRequested;
    unsigned int totNrDevices, deviceID, range;
    unsigned int ranges[100];
};


