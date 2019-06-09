// ROS Libs
#include <ros/ros.h>

// ROS HW interface libs
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

/*
extern "C" {
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

    // CAN libs
    #include <linux/can.h>
    #include <linux/can/raw.h>
}

// CAN commands
#define CAN_PING        1
#define CAN_SET_SPEED   2
#define CAN_GET_SPEED   3

enum SocketCanStatus
{
    STATUS_OK = 0,
    STATUS_SOCKET_CREATE_ERROR = 1,
    STATUS_WRITE_ERROR = 2,
    STATUS_READ_ERROR = 3,
    STATUS_BIND_ERROR = 4,
    STATUS_FALSE_ID = 5,
    STATUS_FALSE_MSG_LEN = 6,
};
*/

class rover_interface : public hardware_interface::RobotHW
{
public:
    rover_interface(const char** joint_names);
    //virtual ~Motor();
    //int initCAN(const char *ifname, unsigned int *can_id, unsigned int own_id, unsigned int read_timeout_ms);
    //int setSpeed();
    //int getSpeed(unsigned int can_id);
    double cmd[4];
    double vel[4];

private:
    int can_socket;
    unsigned int can_timeout;
    //struct sockaddr_can addr;
    //struct can_frame frame;
    //struct ifreq ifr;
    unsigned int own_id;
    unsigned int can_ids[4];

    hardware_interface::JointStateInterface jnt_state_interface;
    //hardware_interface::PositionJointInterface jnt_pos_interface;
    hardware_interface::VelocityJointInterface jnt_vel_interface;

    double prev_cmd[4];
    double pos[4];
    double eff[4];
};
