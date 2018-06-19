#include <ros/ros.h>
#include <thorvald_base/CANFrame.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>


class MagSensNode
{
  public:
  MagSensNode();

  private:
  ros::NodeHandle nh_;
  ros::Subscriber can_sub_;
  //ros::Publisher pow_pub_;
  //ros::Publisher tot_pow_pub_;
  
  void canCallback(const thorvald_base::CANFrameConstPtr& can_msg);




};

MagSensNode::MagSensNode()
{
  can_sub_ = nh_.subscribe("can_frames_device_r", 1, &MagSensNode::canCallback, this);

  //pow_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("corner_power", 1);
  //tot_pow_pub_ = nh_.advertise<std_msgs::Float64>("total_power", 1);

}



void MagSensNode::canCallback(const thorvald_base::CANFrameConstPtr& can_msg)
{
    if (can_msg->id == 404)
    {
      signed char left_marker = can_msg->data[0];
      signed char right_marker = can_msg->data[1];

      std::cout << "left: " << (int)left_marker << ", right: " << (int)right_marker << std::endl;
    }

}


int main (int argc, char** argv)
{
  ros::init(argc, argv, "mag_node");
  MagSensNode mag_sens;
  ros::spin();
  return 0;

}
