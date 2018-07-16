// Ros
#include <ros/ros.h>

// Messages
#include <thorvald_msgs/ThorvaldIO.h>
#include <thorvald_base/EnclosureState.h>
#include <geometry_msgs/Twist.h>

// Services
#include <thorvald_base/CANFrame.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>
#include <semi_auto/PlantMonitoring.h>
#include <semi_auto/AddTwoInts.h>


class SemiAuto
{
public:
    SemiAuto(ros::NodeHandle &nh_);

    // Functions
    bool railDetection();
    bool wallDetection();
    bool floorDetection();
    bool tapeDetection();
    void automaticRailService();
    void turnOnLight();
    void turnOffLight();

    std_srvs::SetBool::Request lightSrv;

private:

    /** Service **/

    bool add(semi_auto::AddTwoInts::Request  &req,
             semi_auto::AddTwoInts::Response &res);

    bool plant_monitoring(std_srvs::Trigger::Request  &req,
             std_srvs::Trigger::Response &res);

    bool plant_light_on(std_srvs::Trigger::Request  &req,
                          std_srvs::Trigger::Response &res);

    bool plant_light_off(std_srvs::Trigger::Request  &req,
                        std_srvs::Trigger::Response &res);

    // Service declaration
    ros::ServiceServer add_ints_service_;
    ros::ServiceServer plant_monitoring_service_;

    ros::ServiceServer plant_light_on_service_; //xbox controller service
    ros::ServiceServer plant_light_off_service_; //xbox controller service

    // Client declaration
    ros::ServiceClient light_on_client_;
    ros::ServiceClient light_off_client_;

    /** Subscriber **/

    ros::Subscriber ultrasonic_sub_;
    ros::Subscriber can_sub_;

    /** Publisher **/

    ros::Publisher vel_pub_;

    /** Callbacks **/

    //Check if connected to rail
    void rangeDetection(const thorvald_msgs::ThorvaldIOConstPtr& serial_msg);

    //can messages callback
    void canCallback(const thorvald_base::CANFrameConstPtr& can_msg);

    /** Variables **/
    bool railDetected, wallDetected, floorDetected, tapeDetected;
    bool forwardMotion, entering, plantServiceRequested, lightIsOn;
    unsigned int totNrDevices, deviceID, range;
    unsigned int ranges[100];

    //Magnetic tape marker
    //Ascii
    signed char left_marker;
    signed char right_marker;
};


