#include "semi_automatic_node.h"


/****************************************************************************/
/***************************** ADDING INTEGERS ******************************/
/****************************************************************************/

bool SemiAuto::add(semi_auto::PlantMonitoring::Request  &req,
                   semi_auto::PlantMonitoring::Response &res){
    res.sum = req.a + req.b;
    ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
    ROS_INFO("sending back response: [%ld]", (long int)res.sum);
    return true;
}

/****************************************************************************/
/***************************** PLANT MONITORING *****************************/
/****************************************************************************/


bool SemiAuto::plant_monitoring(semi_auto::PlantMonitoring::Request  &req,
                                         semi_auto::PlantMonitoring::Response &res){




    return true;
}

/****************************************************************************/
/********************************* CONSTRUCTOR ******************************/
/****************************************************************************/


SemiAuto::SemiAuto(ros::NodeHandle &nh_)
{
    add_ints_service_ = nh_.advertiseService("add_ints",&SemiAuto::add, this);
    ROS_INFO("Ready to add two ints.");

    plant_monitoring_service_ = nh_.advertiseService("plant_monitoring", &SemiAuto::plant_monitoring, this);
    ROS_INFO("Ready for plant monitoring. ");

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
