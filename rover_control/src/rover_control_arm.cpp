#include "rover_control_arm.hpp"

rover_interface::rover_interface(std::string ifaceCAN)
{
    struct sockaddr_can addr;
    struct ifreq ifr;

    if ((can_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
    {
        perror("Error while opening socket");
    }

    strcpy(ifr.ifr_name, ifaceCAN.c_str());
    ioctl(can_socket, SIOCGIFINDEX, &ifr);

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(can_socket, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        perror("Error in socket bind");
    }

    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 1000;
    setsockopt(can_socket, SOL_SOCKET, SO_RCVTIMEO, (const char *)&tv, sizeof tv);

    struct can_filter rfilter;
    rfilter.can_id = 0x2;
    rfilter.can_mask = CAN_EFF_MASK;
    setsockopt(can_socket, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));

    // Motor 1
    motors[0] = rover_trinamic_stepper("joint1", can_socket, &jnt_state_interface, &jnt_pos_interface);
    rover_trinamic_stepper::properties props1 = rover_trinamic_stepper::properties(TMCL_MVP, TMCL_GAP, 200, 256, 0x00)
    motors[0].init(rover_trinamic_stepper::properties)

    // TODO more motors & inits

    registerInterface(&jnt_state_interface);
    registerInterface(&jnt_pos_interface);
}

int rover_interface::receive()
{
    for(rover_motor motor:motors){
        motor.receive();
    }
    return 0;
}

int rover_interface::send()
{
    for(rover_motor motor:motors){
        motor.send();
    }
    return 0;
}
