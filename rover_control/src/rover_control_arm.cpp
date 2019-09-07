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

    hardware_interface::JointStateHandle state_handle("joint1", &pos, &vel, &eff);
    jnt_state_interface.registerHandle(state_handle);
    registerInterface(&jnt_state_interface);

    // connect and register the joint velocity interface
    hardware_interface::JointHandle joint_handle(state_handle, &cmd);
    jnt_pos_interface.registerHandle(joint_handle);
    registerInterface(&jnt_pos_interface);
}

int rover_interface::canUpdate()
{
    struct can_frame frame;

    frame.can_id = 1;
    frame.can_dlc = 8;
    frame.data[0] = 0x06;
    frame.data[1] = 0x01;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = 0x00;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    ::write(can_socket, &frame, sizeof(struct can_frame));

    /* Read a message back from the CAN bus */
    recv(can_socket, &frame, sizeof(frame), NULL);

    pos = ((double)ntohl(*(uint32_t *)(frame.data + 3))) / (256 * 200) * 2 * M_PI;
    return 0;
}

int rover_interface::canPublish()
{
    struct can_frame frame;

    double new_cmd = cmd / (2 * M_PI) * (256 * 200);
    
    uint32_t tmp = new_cmd;
    tmp = htonl(tmp);

    frame.can_id = 1;
    frame.can_dlc = 8;
    frame.data[0] = 0x04;
    frame.data[1] = 0x00;
    frame.data[2] = 0x00;
    *(uint32_t*)&frame.data[3] = tmp;
    ::write(can_socket, &frame, sizeof(struct can_frame));

    recv(can_socket, &frame, sizeof(frame), NULL);

    return 0;
}
