#include <serial/serial.h>
#include "ros/ros.h"
#include <string>
#include "std_msgs/Bool.h"

serial::Serial ser;

void red(const std_msgs::Bool &msg)
{
    if(msg.data)
        ser.write("101");
    else
        ser.write("100");
}

void green(const std_msgs::Bool &msg)
{
    if(msg.data)
        ser.write("121");
    else
        ser.write("120");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "power_board");
 
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    // get device
    std::string device;
    if (!nhp.getParam("device", device))
    {
        ROS_ERROR("Device not set!");
        exit(1);
    }

    ser.setPort(device);
    ser.setBaudrate(115200);
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    ser.setTimeout(to);
    ser.open();

    nh.subscribe("lamp/r", 1, red);
    nh.subscribe("lamp/g", 1, green);
 
    ros::spin();

    return 0;
}
