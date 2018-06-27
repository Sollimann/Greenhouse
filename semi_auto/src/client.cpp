#include "semi_automatic_node.h"
#include <cstdlib>

/****************************************************************************/
/********************************* ADD CLIENT *******************************/
/****************************************************************************/


void add_client(char **argv){

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<semi_auto::AddTwoInts>("add_ints");
  semi_auto::AddTwoInts srv;
  srv.request.a = atoll(argv[1]);
  srv.request.b = atoll(argv[2]);
  if (client.call(srv))
  {
    ROS_INFO("Sum: %ld", (long int)srv.response.sum);
  }
  else
  {
    ROS_ERROR("Failed to call service add_two_ints");
  }
}


/****************************************************************************/
/*********************************  CLIENT *******************************/
/****************************************************************************/
/*
void plant_client(char **argv){

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<std_srvs::Trigger>("plant_monitoring");
  std_srvs::Trigger srv;

  if (client.call(srv))
  {
    ROS_INFO("Success");
    //ROS_INFO("Success ", srv.response.message);
  }
  else
  {
    //ROS_ERROR("Failed to call service plant monitoring");
  }
}
*/
/****************************************************************************/
/******************************** ROS SPECIFIC ******************************/
/****************************************************************************/

int main(int argc, char **argv)
{
  ros::init(argc, argv, "client");
  if (argc != 3)
  {
    ROS_INFO("usage: client X Y");
    return 1;
  }


  //add_client(argv);

  //plant_client(argv);

  return 0;
}
