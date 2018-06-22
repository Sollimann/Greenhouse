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

/****************************************************************************/
/********************************* CONSTRUCTOR ******************************/
/****************************************************************************/

MagTapeNode::MagTapeNode(ros::NodeHandle &nh_)
{

  //Initialize
  Kp_x = 0.009;
  Ki_x = 0.001;
  Kp_z = 0.002;
  integral_fwd = 0;



  //Topics
  ultrasonic_sub_ = nh_.subscribe("serial_io", 1, &MagTapeNode::railDetection, this);
  can_sub_ = nh_.subscribe("can_frames_device_r", 1, &MagTapeNode::canCallback, this);
  mag_pub_ = nh_.advertise<geometry_msgs::Twist>("nav_vel",1);

}

/****************************************************************************/
/**************************** CALCULATE VELOCITY ****************************/
/****************************************************************************/

void MagTapeNode::calc_velocity() {

    //Velocities
    double vx,wz;
    double error = (left_marker + right_marker)/2.0; //Average value
    integral_fwd += dt*error;

    //vx = fabs(error*Kp_x) + Ki_x*integral_fwd;
    vx = 0.07;
    wz = -error*Kp_z;

    //Publish
    base_cmd.linear.x = vx*(1-(fabs(error)/SENSOR_MAX_ERROR));
    base_cmd.angular.z = wz;
    mag_pub_.publish(base_cmd);

}

/****************************************************************************/
/******************************* TAPE DETECTION *****************************/
/****************************************************************************/

bool MagTapeNode::tapeDetection() {

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
/***************************** CALLBACK FUNCTION ****************************/
/****************************************************************************/

void MagTapeNode::canCallback(const thorvald_base::CANFrameConstPtr& can_msg)
{
    if (can_msg->id == 404)
    {
        left_marker = can_msg->data[0];
        right_marker = can_msg->data[1];
        //std::cout << "left: " << (int)left_marker << ", right: " << (int)right_marker << std::endl;


        if (tapeDetection()) {
            calc_velocity();
        //Neither tape or rail detected
        }else{

            //Stay idle
            base_cmd.linear.x = 0;
            base_cmd.angular.z = 0;
            mag_pub_.publish(base_cmd);
        }
    }
}

/****************************************************************************/
/****************************** RAIL DETECTION ******************************/
/****************************************************************************/

void MagTapeNode::railDetection(const thorvald_msgs::ThorvaldIOConstPtr& serial_msg){



    if(serial_msg->analogs.size() > 0 && serial_msg->ranges.size() > 0) {
        std::cout << " analogs : " << serial_msg->analogs[0] << std::endl;
        std::cout << " ranges: " << serial_msg->ranges[0] << std::endl;
    }

}


/****************************************************************************/
/******************************** ROS SPECIFIC ******************************/
/****************************************************************************/

int main (int argc, char** argv)
{
  ros::init(argc, argv, "mag_tape_node");
  ros::NodeHandle nh_;
  MagTapeNode mag_tape(nh_);

    while(ros::ok()) {
        ros::spinOnce();
    }

  return 0;

}
