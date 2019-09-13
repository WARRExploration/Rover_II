#pragma once

// Standard Libs
#include <string>

// ROS Libs
#include <ros/ros.h>

// ROS HW interface libs
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include "controller_manager/controller_manager.h"
#include "hardware_interface/actuator_state_interface.h"
#include <ros/callback_queue.h>

extern "C"
{
// Standard libs
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

// IO Libs
#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <arpa/inet.h>

// CAN libs
#include <linux/can.h>
#include <linux/can/raw.h>
}

// motors
#include <rover_trinamic_stepper.hpp>

class rover_interface_trinamic : public hardware_interface::RobotHW
{
public:
    rover_interface_trinamic(std::string ifaceCAN);
    int receive();
    int send();

private:
    int can_socket;

    hardware_interface::JointStateInterface jnt_state_interface;
    //hardware_interface::PositionJointInterface jnt_pos_interface;
    hardware_interface::PositionJointInterface jnt_pos_interface;

    rover_trinamic_stepper motors[5];
};
