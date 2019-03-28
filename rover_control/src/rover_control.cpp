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

int rover_interface::initCAN(const char *ifname, unsigned int *can_id, unsigned int own_id, unsigned int read_timeout_ms)
{
    this->own_id = own_id;
    can_timeout = read_timeout_ms;

    if((can_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
    {
        // Error while opening socket
        return STATUS_SOCKET_CREATE_ERROR;
    }

    strcpy(ifr.ifr_name, ifname);
    ioctl(can_socket, SIOCGIFINDEX, &ifr);

    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = read_timeout_ms * 1000;  // Not init'ing this can cause strange errors
    setsockopt(can_socket, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv,sizeof(struct timeval));

    if(bind(can_socket, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        // Error in socket bind
        return STATUS_BIND_ERROR;
    }
    this->can_ids[0] = can_ids[0];
    this->can_ids[1] = can_ids[1];
    this->can_ids[2] = can_ids[2];
    this->can_ids[3] = can_ids[3];

    return STATUS_OK;
}

int rover_interface::setSpeed()
{
    for (int i=0; i < 4; i++)
    {
        if (std::fabs(cmd[i] - prev_cmd[i]) > 1E-5)
        {
            //ROS_INFO_STREAM("Prev_cmd: " << prev_cmd[i] << " cmd: " << cmd[i]);
            prev_cmd[i] = cmd[i];
            frame.can_id  = can_ids[i];
            frame.can_dlc = 5;
            frame.data[0] = CAN_SET_SPEED;
            frame.data[1] = 0xFF & (unsigned int) cmd[i];
            frame.data[2] = 0xFF & (((unsigned int) cmd[i]) >> 8);
            frame.data[3] = 0xFF & (((unsigned int) cmd[i]) >> 16);
            frame.data[4] = 0xFF & (((unsigned int) cmd[i]) >> 24);
            if(::write(can_socket, &frame, sizeof(struct can_frame)) != CAN_MTU)
            {
                return STATUS_WRITE_ERROR;
            };
        }
    }
    return STATUS_OK;
}

int rover_interface::getSpeed(unsigned int can_id)
{
    frame.can_id = can_id;
    frame.can_dlc = 1;
    frame.data[0] = CAN_GET_SPEED;
    if (::write(can_socket, &frame, sizeof(struct can_frame)) != CAN_MTU)
    {
        return STATUS_WRITE_ERROR;
    }

    struct can_frame resp;
    int readBytes = ::read(can_socket, &resp, CANFD_MTU);
    if (readBytes != CAN_MTU && readBytes != CANFD_MTU)
    {
        return STATUS_READ_ERROR;
    }
    if (resp.can_id != own_id)
    {
        // Wrong id
        return STATUS_FALSE_ID;
    }
    if (resp.can_dlc != 5)
    {
        // wrong message length
        return STATUS_FALSE_MSG_LEN;
    }
    vel[can_id] = (double) (resp.data[1] || resp.data[2] << 8 || resp.data[3] << 16 || resp.data[4] << 24);

    return STATUS_OK;
}
