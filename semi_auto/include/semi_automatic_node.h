// Ros
#include <ros/ros.h>

// Messages
#include <thorvald_msgs/ThorvaldIO.h>

// Services
#include <thorvald_base/CANFrame.h>
#include <std_srvs/Trigger.h>
#include <semi_auto/PlantMonitoring.h>


class SemiAuto
{
public:
    SemiAuto(ros::NodeHandle &nh_);

private:

    // Service
    bool add(semi_auto::PlantMonitoring::Request  &req,
             semi_auto::PlantMonitoring::Response &res);

    bool plant_monitoring(semi_auto::PlantMonitoring::Request  &req,
             semi_auto::PlantMonitoring::Response &res);

    // Service declaration
    ros::ServiceServer add_ints_service_;
    ros::ServiceServer plant_monitoring_service_;
};


