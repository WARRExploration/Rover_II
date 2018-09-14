#include <motor_driver.h>
#include "ros/ros.h"
#include <exploration_rover_i/tcmc.h>
#include <exploration_rover_i/motor_info.h>
#include <string>
#include <vector>

#define MODE_RS485 1
#define MODE_USB 2

uint8_t mode;
uint8_t usb_address;
motor_driver driver;

XmlRpc::XmlRpcValue v;
int info_framerate;
std::vector<int> info_addresses;
// std::vector<std::vector<int>> infos;
std::vector<int> infos;

void handleInput(const exploration_rover_i::tcmc &msg)
{
    // USB: this message is not for me
    if (mode == MODE_USB && msg.address != usb_address)
        return;

    driver.SendCmd(msg.address, msg.command, msg.type, msg.motor, msg.value);
    ROS_INFO("Motor %x: %d", msg.address, (int)msg.value);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motor_driver");

    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    // get mode param
    std::string mode_str;
    if (!nhp.getParam("mode", mode_str))
    {
        ROS_ERROR("Mode not set!");
        exit(1);
    }

    // set mode
    if (mode_str.compare("usb") == 0)
        mode = MODE_USB;
    else if (mode_str.compare("rs485") == 0)
        mode = MODE_RS485;
    else
    {
        ROS_ERROR("Mode '%s' not defined!", mode_str.c_str());
        exit(1);
    }

    // get device
    std::string device;
    if (!nhp.getParam("device", device))
    {
        ROS_ERROR("Device not set!");
        exit(1);
    }

    // get info_framerate
    if (!nhp.getParam("info_framerate", info_framerate))
    {
        ROS_ERROR("info_framerate not set!");
        exit(1);
    }

    // get info_framerate
    if (!nhp.getParam("info_adresses", info_addresses))
    {
        ROS_ERROR("info_adresses not set!");
        exit(1);
    }

    // get infos
    if (!nhp.getParam("infos", v))
    {
        ROS_ERROR("infos not set!");
        exit(1);
    }

    for (int i = 0; i < v.size(); i++)
    {
        // std::vector<int> info;

        // for (int j = 0; j < v[i].size(); j++)
        // {
        //     info.push_back(v[i][j]);
        // }

        // infos.push_back(info);

        infos.push_back(v[i]);
    }

    // init device
    driver.init(device, 57600);

    // read address of the motor driver
    if (mode == MODE_USB)
    {
        // get global parameter serial address (66)
        driver.SendCmd(0x01, TMCL_GGP, 66, 0, 0);

        UCHAR address, status;
        INT value;
        driver.GetResult(&address, &status, &value);

        // an error occured
        if (status != 100)
        {
            ROS_ERROR("An error occured when trying to get the address\n (Address: %d, Status: %d, Value: %d)", address, status, value);
            exit(1);
        }

        usb_address = value;
        ROS_INFO("Address: %x", usb_address);
    }

    // start the subscriber
    ros::Subscriber input = nh.subscribe("motor_driver", 10, handleInput);
    ros::Publisher pub = nh.advertise<exploration_rover_i::motor_info>("motor_info", 1000);

    // lifecycle
    ros::Rate loop_rate(info_framerate);

    while (ros::ok())
    {
        for (int i = 0; i < infos.size(); i++)
        {
            for (int j = 0; j < info_addresses.size(); j++)
            {
                int address = info_addresses[j];
                int motor = 0;
                int type = infos[i];

                // if (mode == MODE_USB && address != usb_address)
                //     continue;

                // UCHAR r_address, status;
                // INT value;
                // driver.SendCmd(address, 6, type, 0, 0);
                // driver.GetResult(&r_address, &status, &value);
                // // ROS_INFO("Address: %x\t type: %x\t value: %d", usb_address, infos[i], value);

                // // an error occured
                // if (status != 100)
                // {
                //     ROS_ERROR("An error occured when trying to get info from motor %x (status: %x)", address, status);
                //     continue;
                // }

                // exploration_rover_i::motor_info info_msg;
                // info_msg.address = address;
                // info_msg.motor = motor;
                // info_msg.type = type;
                // info_msg.value = value;

                // pub.publish(info_msg);
            }
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}