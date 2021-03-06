#include "semi_automatic_node.h"

/****************************************************************
 *
 * Copyright (c) 2018
 *
 * Norwegian university of Life Sciences (NMBU)
 * Robotics and Control
 *
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Project name: UV lights tomatoes
 * ROS package name: semi_auto
 * Description: This node ...
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Kristoffer Rakstad Solberg, email:kristrso@stud.ntnu.no
 *
 * Date of creation: Juli 2018
 * ToDo: - Clean up
 *
 *
 ****************************************************************/


/****************************************************************************/
/****************************** FLOOR DETECTION *****************************/
/****************************************************************************/

bool SemiAuto::floorDetection() {

    int UPPER_LIMIT = 20;
    int LOWER_LIMIT = 11;
    int count = 0;
    int floorDeviceIndexStart = 0;
    int floorDeviceIndexEnd = 1;

    while(count < 30){

        for(int i = floorDeviceIndexStart; i <= floorDeviceIndexEnd; i++){
            //std::cout << "count: " << count << std::endl;
            if(ranges[i] <= UPPER_LIMIT && ranges[i] >= LOWER_LIMIT){
                count++;
            }else{
                //std::cout << "no rail detection" << std::endl;
                return false;

            }
        }

    }
    //std::cout << "rail detection" << std::endl;
    return true;
}


/****************************************************************************/
/******************************* RAIL DETECTION *****************************/
/****************************************************************************/

bool SemiAuto::railDetection() {

    int UPPER_LIMIT = 10;
    int LOWER_LIMIT = 1;
    int count = 0;
    int railDeviceIndexStart = 0;
    int railDeviceIndexEnd = 1;

    while(count < 30){

        for(int i = railDeviceIndexStart; i <= railDeviceIndexEnd; i++){
            //std::cout << "count: " << count << std::endl;
            if(ranges[i] <= UPPER_LIMIT && ranges[i] >= LOWER_LIMIT){
                count++;
            }else{
                 //std::cout << "no rail detection" << std::endl;
                return false;

            }
        }

    }
    //std::cout << "rail detection" << std::endl;
    return true;
}

/****************************************************************************/
/******************************* WALL DETECTION *****************************/
/****************************************************************************/

bool SemiAuto::wallDetection() {

    int UPPER_LIMIT = 50;
    int LOWER_LIMIT = 10;
    int count = 0;
    int wallDeviceIndexStart = 2;
    int wallDeviceIndexEnd = totNrDevices-1;

    while(count < 50){

        for(int i = wallDeviceIndexStart; i <= wallDeviceIndexEnd; i++){
            if(ranges[i] <= UPPER_LIMIT && ranges[i] >= LOWER_LIMIT){
                count++;
            }else{
                return false;

            }
        }

    }
    return true;
}

/****************************************************************************/
/******************************* TAPE DETECTION *****************************/
/****************************************************************************/

bool SemiAuto::tapeDetection() {

    int count = 0;
    while(left_marker == 0 && right_marker == 0) {
        count++;
        if(count > 30) {
            return false;
        }
    }
    return true;
}

/****************************************************************************/
/************************* RANGE DETECTION CALLBACK *************************/
/****************************************************************************/

void SemiAuto::rangeDetection(const thorvald_msgs::ThorvaldIOConstPtr& serial_msg){

    if(serial_msg->analogs.size() > 0 && serial_msg->ranges.size() > 0) {
        totNrDevices = serial_msg->analogs[1];
        //deviceID = serial_msg->analogs[0];

        for(int i = 0; i < (totNrDevices); i++) {
            range = serial_msg->ranges[i];
            ranges[i] = range;
        }

        std::cout << "range 0: " << ranges[0] << std::endl;
        std::cout << "range 1: " << ranges[1] << std::endl;
        std::cout << "range 2: " << ranges[2] << std::endl;
        std::cout << "range 3: " << ranges[3] << std::endl;
        //std::cout << "left marker: " << left_marker << "  right marker: " << right_marker << std::endl;
    }

    railDetected = railDetection();
    std::cout << "Rail detected: " << railDetected << std::endl;
    wallDetected = wallDetection();
    std::cout << "Wall detected: " << wallDetected << std::endl;
    floorDetected = floorDetection();
    std::cout << "Floor detected: " << floorDetected << std::endl;
    tapeDetected = tapeDetection();
    std::cout << "Tape detected: " << tapeDetected << std::endl;



    if(plantServiceRequested){
        // Start service
        automaticRailService();
    }

}

/****************************************************************************/
/**************************** CAN MSG CALLBACK  *****************************/
/****************************************************************************/

void SemiAuto::canCallback(const thorvald_base::CANFrameConstPtr& can_msg)
{
    if (can_msg->id == 404) {
        left_marker = can_msg->data[0];
        right_marker = can_msg->data[1];
        //std::cout << "left: " << (int)left_marker << ", right: " << (int)right_marker << std::endl;
    }

}


/****************************************************************************/
/************************** AUTOMATIC RAIL SERVICE **************************/
/****************************************************************************/

void SemiAuto::automaticRailService(){

   // railDetected = railDetection();
    //Velocities
    double vx;
    vx = 0.5;
    geometry_msgs::Twist base_cmd;

    // No rail detected
    if(!railDetected) {
        //Publish
        base_cmd.linear.x = 0;
        vel_pub_.publish(base_cmd);
    }

    // Rail and tape detected
    if(railDetected && forwardMotion && tapeDetected) {
        //Publish
        base_cmd.linear.x = 0.3*vx;
        vel_pub_.publish(base_cmd);
        entering = true;
    // Rail has successfully been entered
    }else if(entering && railDetected && forwardMotion && !tapeDetected){
        //Publish
        base_cmd.linear.x = vx;
        vel_pub_.publish(base_cmd);
    }

    // wall detected
    if(railDetected && wallDetected && forwardMotion) {

        //Publish
        base_cmd.linear.x = 0;
        vel_pub_.publish(base_cmd);

        //De-acceleration
        ros::Rate r(1); // 10 hz
        r.sleep();

        //Publish
        base_cmd.linear.x = vx;
        vel_pub_.publish(base_cmd);

        //Returning
        forwardMotion = false;
        entering = false;
    }

    // Returning
    if(railDetected && !forwardMotion) {
        //Publish
        base_cmd.linear.x = -vx;
        vel_pub_.publish(base_cmd);

        // When exiting the rails backwards
    }else if(!forwardMotion && floorDetected && tapeDetected){

        // Make forward motion true for next run
        forwardMotion = true;

        // Service complete
        plantServiceRequested = false;
    }


}

/****************************************************************************/
/***************************** ADDING INTEGERS ******************************/
/****************************************************************************/

bool SemiAuto::add(semi_auto::AddTwoInts::Request  &req,
                   semi_auto::AddTwoInts::Response &res){
    res.sum = req.a + req.b;
    ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
    ROS_INFO("sending back response: [%ld]", (long int)res.sum);
    return true;
}

/****************************************************************************/
/***************************** PLANT MONITORING *****************************/
/****************************************************************************/


bool SemiAuto::plant_monitoring(std_srvs::Trigger::Request  &req,
                                         std_srvs::Trigger::Response &res){

    ROS_INFO("Service code entered");

    if(railDetected && tapeDetected){
        // Service response
        res.success = true;
        res.message = "Rail and tape Detected, start service";
        forwardMotion = true;
        plantServiceRequested = true;
    }else{
        res.success = false;
        res.message = "Rail and tape not detected, abort service";
    }

    return true;
}

/****************************************************************************/
/*************************** PLANT LIGHT SERVICE ****************************/
/****************************************************************************/

//On

bool SemiAuto::plant_light_on(std_srvs::Trigger::Request  &req,
                                std_srvs::Trigger::Response &res){

    ROS_INFO("Service code entered");
        // Service response
        res.success = true;
        res.message = "Lights available, start service";

    turnOnLight();

    return true;
}


void SemiAuto::turnOnLight(){

    std_srvs::SetBool lightSrv;
    lightSrv.request.data = true;
    lightSrv.response.success = true;
    lightSrv.response.message = "Lights on";


    ROS_INFO("calling service: light_service_on ");
    if (light_on_client_.call(lightSrv))
    {
        ROS_INFO("finished calling");
    }
    else
    {
        ROS_INFO("calling service failed");
    }

}

//Off

bool SemiAuto::plant_light_off(std_srvs::Trigger::Request  &req,
                              std_srvs::Trigger::Response &res){

    ROS_INFO("Service code entered");
    // Service response
    res.success = true;
    res.message = "Lights available, start service";

    turnOffLight();

    return true;
}


void SemiAuto::turnOffLight(){

    std_srvs::SetBool lightSrv;
    lightSrv.request.data = false;
    lightSrv.response.success = true;
    lightSrv.response.message = "Lights off";

    ROS_INFO("calling service: light_service_off");
    if (light_off_client_.call(lightSrv))
    {
        ROS_INFO("finished calling");
    }
    else
    {
        ROS_INFO("calling service failed");
    }

}


/****************************************************************************/
/********************************* CONSTRUCTOR ******************************/
/****************************************************************************/


SemiAuto::SemiAuto(ros::NodeHandle &nh_)
{
    /** Services **/
    add_ints_service_ = nh_.advertiseService("add_ints",&SemiAuto::add, this);
    ROS_INFO("Ready to add two ints.");

    plant_monitoring_service_ = nh_.advertiseService("plant_monitoring", &SemiAuto::plant_monitoring, this);
    ROS_INFO("Ready for plant monitoring. ");

    plant_light_on_service_ = nh_.advertiseService("plant_light_on_service", &SemiAuto::plant_light_on, this);
    ROS_INFO("Ready for plant light on. ");

    plant_light_off_service_ = nh_.advertiseService("plant_light_off_service", &SemiAuto::plant_light_off, this);
    ROS_INFO("Ready for plant light off. ");

    /** Clients **/

    light_on_client_ = nh_.serviceClient<std_srvs::SetBool>("switch_rl3"); //write to allready existing service
    light_off_client_ = nh_.serviceClient<std_srvs::SetBool>("switch_rl3");

    /** Topics **/
    ultrasonic_sub_ = nh_.subscribe("serial_io", 1, &SemiAuto::rangeDetection, this);
    can_sub_ = nh_.subscribe("can_frames_device_r", 1, &SemiAuto::canCallback, this);
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("nav_vel",1);


    /** Variables **/

    plantServiceRequested = false;
    forwardMotion = true;
    wallDetected = false;
    floorDetected = true;
    tapeDetected = false;
    entering = false;
    lightIsOn = false;


}



/****************************************************************************/
/******************************** ROS SPECIFIC ******************************/
/****************************************************************************/

int main (int argc, char** argv)
{
  ros::init(argc, argv, "semi_automatic_node");
  ros::NodeHandle nh_;

  // create object
  SemiAuto SemiAutos(nh_);

    while(ros::ok()) {
        ros::spinOnce();
    }

  return 0;

}
