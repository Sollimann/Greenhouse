#include "semi_automatic_node.h"

/****************************************************************************/
/******************************* RAIL DETECTION *****************************/
/****************************************************************************/

bool SemiAuto::railDetection() {

    int UPPER_LIMIT = 15;
    int LOWER_LIMIT = 2;
    int count = 0;

    while(count < 30){

        for(int i = 0; i < totNrDevices; i++){
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
/****************************** RANGE DETECTION ******************************/
/****************************************************************************/

void SemiAuto::rangeDetection(const thorvald_msgs::ThorvaldIOConstPtr& serial_msg){

    if(serial_msg->analogs.size() > 0 && serial_msg->ranges.size() > 0) {
        totNrDevices = serial_msg->analogs[1];
        //deviceID = serial_msg->analogs[0];

        for(int i = 0; i < (totNrDevices); i++) {
            range = serial_msg->ranges[i];
            ranges[i] = range;
        }

        //std::cout << "range 0: " << ranges[0] << std::endl;
        //std::cout << "range 1: " << ranges[1] << std::endl;
        //std::cout << "range 2: " << ranges[2] << std::endl;
    }

    railDetected = railDetection();
    std::cout << "Rail detected: " << railDetected << std::endl;
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
        res.message = "Rail Detected, succsessful service";
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
