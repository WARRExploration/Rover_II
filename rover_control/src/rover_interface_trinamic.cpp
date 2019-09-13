#include <rover_interface_trinamic.hpp>

rover_interface_trinamic::rover_interface_trinamic(std::string ifaceCAN)
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
    motors[0] = rover_trinamic_stepper("joint0", can_socket, &jnt_state_interface, &jnt_pos_interface);
    rover_trinamic_stepper::properties props1 = rover_trinamic_stepper::properties(TMCL_MVP, TMCL_GAP, 200, 16, 0x00);
    motors[0].init(props1);

    // Motor 2
    motors[1] = rover_trinamic_stepper("joint2", can_socket, &jnt_state_interface, &jnt_pos_interface);
    rover_trinamic_stepper::properties props2 = rover_trinamic_stepper::properties(TMCL_MVP, TMCL_GAP, 200, 16, 0x00);
    motors[1].init(props2);

    // Motor 3
    motors[2] = rover_trinamic_stepper("joint3", can_socket, &jnt_state_interface, &jnt_pos_interface);
    rover_trinamic_stepper::properties props3 = rover_trinamic_stepper::properties(TMCL_MVP, TMCL_GAP, 200, 16, 0x00);
    motors[2].init(props3);

    // Motor 4
    motors[3] = rover_trinamic_stepper("joint_hand", can_socket, &jnt_state_interface, &jnt_pos_interface);
    rover_trinamic_stepper::properties props4 = rover_trinamic_stepper::properties(TMCL_MVP, TMCL_GAP, 200, 16, 0x01);
    motors[3].init(props4);

    // Motor 5
    motors[4] = rover_trinamic_stepper("joint_gripper", can_socket, &jnt_state_interface, &jnt_pos_interface);
    rover_trinamic_stepper::properties props5 = rover_trinamic_stepper::properties(TMCL_MVP, TMCL_GAP, 200, 16, 0x02);
    motors[4].init(props5);

    registerInterface(&jnt_state_interface);
    registerInterface(&jnt_pos_interface);
}

int rover_interface_trinamic::receive()
{
    for(rover_trinamic_stepper motor:motors){
        motor.receive();
    }
    return 0;
}

int rover_interface_trinamic::send()
{
    for(rover_trinamic_stepper motor:motors){
        motor.send();
    }
    return 0;
}
