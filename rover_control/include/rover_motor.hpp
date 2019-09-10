#pragma once

// Standard Libs
#include <string>

// ROS HW interface libs
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include "controller_manager/controller_manager.h"
#include "hardware_interface/actuator_state_interface.h"
#include <ros/callback_queue.h>

class rover_motor
{
public:
    rover_motor(std::string joint_name, int can_socket, 
        hardware_interface::JointStateInterface *jnt_state_interface, 
        hardware_interface::PositionJointInterface *jnt_pos_interface);

    virtual int send() = 0;
    virtual int receive() = 0;

private:
    int can_socket;
ri
    double cmd;
    double vel;
    double pos;
    double eff;
};