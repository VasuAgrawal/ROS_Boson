#include "ros/ros.h"
#include "std_msgs/String.h"
#include "flir_boson/RawCommand.h"

#include <sstream>

void rebootCallback(const std_msgs::String::ConstPtr& msg)
{
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<flir_boson::RawCommand>("raw_command");
  flir_boson::RawCommand srv;
  srv.request.command = "00050010";
  srv.request.data = "";

  if (client.call(srv))
  {
    ROS_INFO("Rebooting Boson Core");
  }
  else
  {
    ROS_ERROR("Failed to call service raw_command");
  }
}

void colorpaletteCallback(const std_msgs::String::ConstPtr& msg)
{
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<flir_boson::RawCommand>("raw_command");
  flir_boson::RawCommand srv;
  srv.request.command = "000B0003";
  srv.request.data = msg->data;

  if (client.call(srv))
  {
    ROS_INFO("New LUT ID: [%s]", msg->data.c_str());
  }
  else
  {
    ROS_ERROR("Failed to call service raw_command");
  }
}

int main(int argc, char **argv)
{
  std::stringstream response;
  std_msgs::String msg_serialnumber;
  std_msgs::String msg_partnumber;
  std_msgs::String msg_fwversion;
  std_msgs::String msg_coretemp;
  std_msgs::String msg_lutid;

  ros::init(argc, argv, "boson_driver");

  ros::NodeHandle n;

  //Publisheres
  ros::Publisher serialnumber_pub = n.advertise<std_msgs::String>("flir/state/thermal/serialnumber", 1000);
  ros::Publisher partnumber_pub = n.advertise<std_msgs::String>("flir/state/thermal/partnumber", 1000);
  ros::Publisher fwversion_pub = n.advertise<std_msgs::String>("flir/state/thermal/fwversion", 1000);
  ros::Publisher coretemp_pub = n.advertise<std_msgs::String>("flir/state/thermal/coretemperature", 1000);
  ros::Publisher lutid_pub = n.advertise<std_msgs::String>("flir/state/thermal/colorpalette", 1000);

  //Subscriberes
  ros::Subscriber reboot_sub = n.subscribe("flir/command/thermal/reboot", 1000, rebootCallback);
  ros::Subscriber colorpalette_sub = n.subscribe("flir/command/thermal/colorpalette", 1000, colorpaletteCallback);

  ros::ServiceClient client = n.serviceClient<flir_boson::RawCommand>("raw_command");
  flir_boson::RawCommand srv;
  ros::Rate loop_rate(0.1);

  //Serial Number
  srv.request.command = "00050002";
  srv.request.data = "";

  if (client.call(srv))
  {
    response.str("");
    response << "Serial Number: " << srv.response.output;
    msg_serialnumber.data = response.str();
  }
  else
  {
    ROS_ERROR("Failed to call service raw_command: serialnumber");
    return 1;
  }

  //Part Number
  srv.request.command = "00050004";
  srv.request.data = "";

  if (client.call(srv))
  {
    response.str("");
    response << "Part Number: " << srv.response.output;
    msg_partnumber.data = response.str();
  }
  else
  {
    ROS_ERROR("Failed to call service raw_command: partnumber");
    return 1;
  }

  //FW version
  srv.request.command = "00050022";
  srv.request.data = "";

  if (client.call(srv))
  {
    response.str("");
    response << "FW: " << srv.response.output;
    msg_fwversion.data = response.str();
  }
  else
  {
    ROS_ERROR("Failed to call service raw_command: fwversion");
    return 1;
  }

  //Core Temperature
  srv.request.command = "00050068";
  srv.request.data = "";

  if (client.call(srv))
  {
    response.str("");
    response << "Core Temperature: " << srv.response.output;
    msg_coretemp.data = response.str();
  }
  else
  {
    ROS_ERROR("Failed to call service raw_command: coretemperature");
    return 1;
  }

  while (ros::ok())
  {

    //Color Palette
    srv.request.command = "000B0004";
    srv.request.data = "";

    if (client.call(srv))
    {
      response.str("");
      response << "LUT ID: " << srv.response.output;
      msg_lutid.data = response.str();
      ROS_INFO("%s", msg_lutid.data.c_str());
      lutid_pub.publish(msg_lutid);
    }
    else
    {
      ROS_ERROR("Failed to call service raw_command: lut id");
    }

    ROS_INFO("%s", msg_serialnumber.data.c_str());
    ROS_INFO("%s", msg_partnumber.data.c_str());
    ROS_INFO("%s", msg_fwversion.data.c_str());
    ROS_INFO("%s", msg_coretemp.data.c_str());

    serialnumber_pub.publish(msg_serialnumber);
    partnumber_pub.publish(msg_partnumber);
    fwversion_pub.publish(msg_fwversion);
    coretemp_pub.publish(msg_coretemp);
    
    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}

