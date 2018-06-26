#include "semi_automatic_node.h"
#include <cstdlib>

/****************************************************************************/
/********************************* ADD CLIENT *******************************/
/****************************************************************************/


void add_client(char **argv){

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<semi_auto::PlantMonitoring>("add_ints");
  semi_auto::PlantMonitoring srv;
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


  add_client(argv);

  return 0;
}
