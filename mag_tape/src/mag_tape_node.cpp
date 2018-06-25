#include "mag_tape_node.h"

/****************************************************************************/
/********************************* CONSTRUCTOR ******************************/
/****************************************************************************/

MagTape::MagTape(ros::NodeHandle &nh_)
{

  //Initialize
  Kp_x = 0.009;
  Ki_x = 0.001;
  Kp_z = 0.002;
  integral_fwd = 0;

  //Boolians
  railDetected = false;
  tapeDetected = false;
  enterRail = true;
  leaveRail = false;
  railMonitoringMission = false;
  initialize_range_array = true;

  //Devices
  totNrDevices = 1;
  currentNrDevices = 0;

  //RailWay Object
  //RailWay rails;

  //Topics
  ultrasonic_sub_ = nh_.subscribe("serial_io", 1, &MagTape::railDetection, this);
  can_sub_ = nh_.subscribe("can_frames_device_r", 1, &MagTape::canCallback, this);
  mag_pub_ = nh_.advertise<geometry_msgs::Twist>("nav_vel",1);

}

/****************************************************************************/
/**************************** CALCULATE VELOCITY ****************************/
/****************************************************************************/

void MagTape::calc_velocity_along_tape() {

    //Velocities
    double vx,wz;
    error = (left_marker + right_marker)/2.0; //Average value
    integral_fwd += dt*error;

    //vx = fabs(error*Kp_x) + Ki_x*integral_fwd;
    vx = 0.17;
    wz = -error*Kp_z;

    //Publish
    base_cmd.linear.x = vx*(1-(fabs(error)/SENSOR_MAX_ERROR));
    base_cmd.angular.z = wz;
    mag_pub_.publish(base_cmd);

}

/****************************************************************************/
/***************************** PLANT MONITORING *****************************/
/****************************************************************************/


bool RailWay::automaticPlantMonitoring(){

    bool monitoring = true;



        // Railings have been detected and we want to enter
        if (enterRail) {

            std::cout << "Enter the rail " << std::endl;

            if (tapeDetected && railDetected){
                // When the tape is detected on the way back
                // Leave the rail

                std::cout << "End of rail detected " << std::endl;
                enterRail = false;
                leaveRail = true;
            }
            //monitoring = true;
        }

        // End of railings have been detected and we want to exit
        if (leaveRail) {

            std::cout << "Leave the rail... " << std::endl;


            if(this->tapeDetected && !this->railDetected){
            // If the robot has left the rails and only detect the magnetic tape
                enterRail = true;
                leaveRail = false;

                std::cout << "Rails have been left " << std::endl;

                monitoring = false;
        }
    }

return monitoring;
}


/****************************************************************************/
/******************************** CONTROLLER  *******************************/
/****************************************************************************/

void MagTape::controller(){

    if(tapeDetected && railDetected || railMonitoringMission){

        //railMonitoringMission = automaticPlantMonitoring();


        //std::cout << "both rail and tape detected" << std::endl;

    }else if(tapeDetected || railDetected){
        if (tapeDetected) {
            calc_velocity_along_tape();
            //Neither tape or rail detected
            //std::cout << "tape detected " << std::endl;
        }
        if (railDetected) {
            //calc_velocty_along_rail();
            //std::cout << "rail detected " << std::endl;
        }
    }else{
        //Stay idle
        std::cout << "No speed" << std::endl;
        base_cmd.linear.x = 0;
        base_cmd.angular.z = 0;
        mag_pub_.publish(base_cmd);
    }

}

/****************************************************************************/
/******************************* TAPE DETECTION *****************************/
/****************************************************************************/

bool MagTape::tapeDetection() {

    int count = 0;
    while(left_marker == 0 && right_marker == 0) {
        count++;
        if(count > 30) {
            //std::cout << "No tape detected" << std::endl;
            return false;
        }

    }

    return true;
}


/****************************************************************************/
/***************************** CAN MSG CALLBACK  ****************************/
/****************************************************************************/

void MagTape::canCallback(const thorvald_base::CANFrameConstPtr& can_msg)
{
    if (can_msg->id == 404) {
        left_marker = can_msg->data[0];
        right_marker = can_msg->data[1];
        //std::cout << "left: " << (int)left_marker << ", right: " << (int)right_marker << std::endl;

        tapeDetected = tapeDetection();
    }

}

/****************************************************************************/
/****************************** RAIL DETECTION ******************************/
/****************************************************************************/

void MagTape::railDetection(const thorvald_msgs::ThorvaldIOConstPtr& serial_msg){

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

        //std::cout << "range 0: " << serial_msg->ranges[0] << std::endl;
        //std::cout << "range 1: " << serial_msg->ranges[1] << std::endl;
        //std::cout << "range 2: " << serial_msg->ranges[2] << std::endl;


        for(int i = 0; i < totNrDevices; i++){
            //std::cout << "ranges " << i << "is equal to: " << ranges[i] << std::endl;
            if(ranges[i] < 15 && ranges[i] > 4){
                railDetected = true;
            }else{
                railDetected = false;
                //std::cout << "No rail" << std::endl;
            }

        }
    }

    controller();
}


/****************************************************************************/
/******************************** ROS SPECIFIC ******************************/
/****************************************************************************/

int main (int argc, char** argv)
{
  ros::init(argc, argv, "mag_tape_node");
  ros::NodeHandle nh_;
  MagTape mag_tape(nh_);

    while(ros::ok()) {
        ros::spinOnce();
    }

  return 0;

}
