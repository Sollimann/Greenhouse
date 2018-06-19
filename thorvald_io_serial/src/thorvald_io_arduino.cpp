#include <ros/ros.h>
#include <serial/serial.h>
#include <thorvald_msgs/ThorvaldIO.h>
//#include <std_msgs/Int16.h>
// ROS service includes
#include <sstream>


class LightRigSerial
{

  public:
  LightRigSerial(std::string port, int baud, int serial_timeout);
  void requestStatus();

  private:
  serial::Serial serial_;

  ros::NodeHandle nh_;
  ros::Publisher status_pub_;

  std::string sendSerialCommand(std::string serial_command);
  
};


LightRigSerial::LightRigSerial(std::string port, int baud, int serial_timeout) :
  serial_(port, baud, serial::Timeout::simpleTimeout(serial_timeout))

{


  // subscribe to topics

  // publish to topics
  status_pub_ = nh_.advertise<thorvald_msgs::ThorvaldIO>("serial_io", 1);

  // advertise services

  std::cout << "Is the serial port open?";
  
  if(serial_.isOpen())
    std::cout << " Yes." << std::endl;
  else
    std::cout << " No." << std::endl;
 
}


void LightRigSerial::requestStatus()
{
  
  thorvald_msgs::ThorvaldIO status_msg;
  status_msg.header.stamp = ros::Time::now();
  
  status_msg.digitals.resize(2);
  std::string end_stops_str;
  end_stops_str = sendSerialCommand("?e:,");
  end_stops_str = end_stops_str.substr(end_stops_str.find(':')+1, end_stops_str.find(',')-end_stops_str.find(':')-1);

  std::istringstream end_stops_ss(end_stops_str);
  int d1, d2;
  end_stops_ss >> d1 >> d2;

  status_msg.digitals[0] = d1;
  status_msg.digitals[1] = d2;


  int n = 0;
  bool finished = false;

  std::string dists_str;

  dists_str = sendSerialCommand("?c:,");
  dists_str = dists_str.substr(dists_str.find('~')+1, dists_str.find(',')-1);
  dists_str += " ,";
  
  while(!finished && n < 10)
  {
    if (dists_str[0] == ',')
    {
      finished = true;
    }
    else
    {
      size_t pos = dists_str.find(' ');
      int dist = atoi(dists_str.substr(0, pos).c_str());
      status_msg.ranges.push_back(dist);
      dists_str.erase(0, pos+1);
      n++;

    }


  }


  status_msg.ranges.resize(n);


  status_pub_.publish(status_msg);

}




std::string LightRigSerial::sendSerialCommand(std::string serial_command)
{
  //ROS_INFO("sending: %s", serial_command.c_str());
  serial_.write(serial_command);
  std::string reply = serial_.readline(1000, ",\r\n");
  reply = reply.substr(0, reply.size()-2);
  //ROS_INFO("reply: %s", reply.c_str());
  
  return reply;
}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "light_rig_serial");


  if(argc < 2) {
    ROS_WARN("Usage: uv_rig_serial <serial port address> <baudrate>"); 
    return 1;
  }


  std::string port(argv[1]); 

  unsigned long baud = 0;
  sscanf(argv[2], "%lu", &baud);


  LightRigSerial light_rig_serial(port, baud, 1000);

  ros::Rate rate(20);
  ros::Duration(3).sleep();

  while(ros::ok())
  {
    ros::spinOnce();
    light_rig_serial.requestStatus();
    rate.sleep();
  }

  return 1;

}

