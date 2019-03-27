#include "rover_control.hpp"
/*extern "C" {
    ssize_t write(int, const void *, size_t);
}*/


rover_interface::rover_interface(const char** joint_names)
{
    // connect and register the joint state interface
    hardware_interface::JointStateHandle state_handle_0(joint_names[0], &pos[0], &vel[0], &eff[0]);
    hardware_interface::JointStateHandle state_handle_1(joint_names[1], &pos[1], &vel[1], &eff[1]);
    hardware_interface::JointStateHandle state_handle_2(joint_names[2], &pos[2], &vel[2], &eff[2]);
    hardware_interface::JointStateHandle state_handle_3(joint_names[3], &pos[3], &vel[3], &eff[3]);
    jnt_state_interface.registerHandle(state_handle_0);
    jnt_state_interface.registerHandle(state_handle_1);
    jnt_state_interface.registerHandle(state_handle_2);
    jnt_state_interface.registerHandle(state_handle_3);
    registerInterface(&jnt_state_interface);

    // connect and register the joint position interface
    //hardware_interface::JointHandle pos_handle(jnt_state_interface.getHandle(motor_name), &cmd);
    //jnt_pos_interface.registerHandle(pos_handle);
    //registerInterface(&jnt_pos_interface);

    // connect and register the joint velocity interface
    hardware_interface::JointHandle vel_handle_0(jnt_state_interface.getHandle(joint_names[0]), &cmd[0]);
    hardware_interface::JointHandle vel_handle_1(jnt_state_interface.getHandle(joint_names[1]), &cmd[1]);
    hardware_interface::JointHandle vel_handle_2(jnt_state_interface.getHandle(joint_names[2]), &cmd[2]);
    hardware_interface::JointHandle vel_handle_3(jnt_state_interface.getHandle(joint_names[3]), &cmd[3]);
    jnt_vel_interface.registerHandle(vel_handle_0);
    jnt_vel_interface.registerHandle(vel_handle_1);
    jnt_vel_interface.registerHandle(vel_handle_2);
    jnt_vel_interface.registerHandle(vel_handle_3);
    registerInterface(&jnt_vel_interface);
}

int rover_interface::initCAN(const char *ifname, unsigned int *can_id)
{
    if((can_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
    {
        perror("Error while opening socket");
        return -1;
    }

    strcpy(ifr.ifr_name, ifname);
    ioctl(can_socket, SIOCGIFINDEX, &ifr);

    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if(bind(can_socket, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        perror("Error in socket bind");
        return -2;
    }
    this->can_id[0] = can_id[0];
    this->can_id[1] = can_id[1];
    this->can_id[2] = can_id[2];
    this->can_id[3] = can_id[3];
}

int rover_interface::setSpeed()
{
    unsigned int totalBytes = 0;
    for (int i=0; i < 4; i++)
    {
        frame.can_id  = can_id[i];
        frame.can_dlc = 4;
        frame.data[0] = 0xFF & (unsigned int) cmd[i];
        frame.data[1] = 0xFF & (((unsigned int) cmd[i]) >> 8);
        frame.data[2] = 0xFF & (((unsigned int) cmd[i]) >> 16);
        frame.data[3] = 0xFF & (((unsigned int) cmd[i]) >> 24);
        totalBytes += ::write(can_socket, &frame, sizeof(struct can_frame));
    }
    //printf("Wrote %d bytes\n", nbytes);
    return totalBytes;
}
