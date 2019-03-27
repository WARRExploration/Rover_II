// ROS HW interface libs
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

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

class rover_interface : public hardware_interface::RobotHW
{
public:
    rover_interface(const char** joint_names);
    //virtual ~Motor();
    int initCAN(const char *ifname, unsigned int *can_id);
    int setSpeed();

private:
    int can_socket;
    struct sockaddr_can addr;
    struct can_frame frame;
    struct ifreq ifr;
    double can_id[4];

    hardware_interface::JointStateInterface jnt_state_interface;
    //hardware_interface::PositionJointInterface jnt_pos_interface;
    hardware_interface::VelocityJointInterface jnt_vel_interface;
    double cmd[4];
    double pos[4];
    double vel[4];
    double eff[4];
};
