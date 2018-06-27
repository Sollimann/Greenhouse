#include "semi_automatic_node.h"


/****************************************************************************/
/****************************** FLOOR DETECTION *****************************/
/****************************************************************************/

bool SemiAuto::floorDetection() {

    int UPPER_LIMIT = 25;
    int LOWER_LIMIT = 20;
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

    int UPPER_LIMIT = 19;
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
/****************************** RANGE DETECTION *****************************/
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
    }

    railDetected = railDetection();
    std::cout << "Rail detected: " << railDetected << std::endl;
    wallDetected = wallDetection();
    std::cout << "Wall detected: " << wallDetected << std::endl;
    floorDetected = floorDetection();
    std::cout << "Floor detected: " << floorDetected << std::endl;



    if(plantServiceRequested){
        // Start service
        automaticRailService();
    }

}


/****************************************************************************/
/************************** AUTOMATIC RAIL SERVICE **************************/
/****************************************************************************/

void SemiAuto::automaticRailService(){

   // railDetected = railDetection();
    //Velocities
    double vx;
    vx = 0.25;
    geometry_msgs::Twist base_cmd;

    // No rail detected
    if(!railDetected) {
        //Publish
        base_cmd.linear.x = 0;
        vel_pub_.publish(base_cmd);
    }

    // Rail detected
    if(railDetected && forwardMotion) {
        //Publish
        base_cmd.linear.x = vx;
        vel_pub_.publish(base_cmd);
    }

    // wall detected
    if(railDetected && wallDetected && forwardMotion) {

        //De-acceleration
        for(int i = 1; i < 100;i++){

            //Publish
            base_cmd.linear.x = vx/1.1;
            vel_pub_.publish(base_cmd);
        }

        // bool
        forwardMotion = false;
    }

    // Returning
    if(railDetected && !forwardMotion) {
        //Publish
        base_cmd.linear.x = -vx;
        vel_pub_.publish(base_cmd);
    }

    // When exiting the rails backwards
    if(!forwardMotion && floorDetected){

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

    if(railDetected){
        // Service response
        res.success = true;
        res.message = "Rail Detected, start service";
        plantServiceRequested = true;
    }else{
        res.success = false;
        res.message = "Rail not detected, abort service";
    }


    return true;
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

    /** Topics **/
    ultrasonic_sub_ = nh_.subscribe("serial_io", 1, &SemiAuto::rangeDetection, this);
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("nav_vel",1);


    /** Variables **/

    plantServiceRequested = false;
    forwardMotion = true;
    wallDetected = false;
    floorDetected = true;

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
