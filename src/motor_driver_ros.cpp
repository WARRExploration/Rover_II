#include <motor_driver.h>
#include "ros/ros.h"
#include <exploration_rover_i/motor_ror.h>
#include <string>

#define MODE_RS485 1
#define MODE_USB 2

uint8_t mode;
uint8_t usb_address;
motor_driver driver;

void handleInput(const exploration_rover_i::motor_ror &msg)
{
    // USB: this message is not for me
    if(mode == MODE_USB && msg.address != usb_address) return;

    driver.SendCmd(msg.address, TMCL_ROR, 0, 0, msg.value);
    ROS_INFO("Motor %x: %d", msg.address, (int)msg.value);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motor_driver");

    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    // get mode param
    std::string mode_str;
    if(!nhp.getParam("mode", mode_str))
    {
        ROS_ERROR("Mode not set!");
        exit(1);
    }

    // set mode
    if (mode_str.compare("usb") == 0) mode = MODE_USB;
    else if (mode_str.compare("rs485") == 0) mode = MODE_RS485;
    else
    {
        ROS_ERROR("Mode '%s' not defined!", mode_str.c_str());
        exit(1);
    }

    // get device
    std::string device;
    if(!nhp.getParam("device", device))
    {
        ROS_ERROR("Device not set!");
        exit(1);
    }

    // init device
    driver.init(device, 57600);

    // read address of the motor driver
    if(mode == MODE_USB)
    {
        // get global parameter serial address (66)
        driver.SendCmd(0x01, TMCL_GGP, 66, 0, 0);

        UCHAR address, status;
        INT value;
        driver.GetResult(&address, &status, &value);

        // an error occured
        if(status != 100)
        {
            ROS_ERROR("An error occured when trying to get the address\n (Address: %d, Status: %d, Value: %d)", address, status, value);
            exit(1);
        }

        usb_address = value;
        ROS_INFO("Address: %x", usb_address);
    }

    // start the subscriber
    ros::Subscriber input = nh.subscribe("motor_driver", 10, handleInput);

    // lifecycle
    ros::spin();

    return 0;
}