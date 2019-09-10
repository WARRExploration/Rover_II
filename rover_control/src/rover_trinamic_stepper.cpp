# include <rover_trinamic_stepper.hpp>

rover_trinamic_stepper::rover_trinamic_stepper(std::string joint_name, int can_socket, 
    hardware_interface::JointStateInterface *jnt_state_interface, 
    hardware_interface::PositionJointInterface *jnt_pos_interface) 
    : rover_motor(joint_name, can_socket, jnt_state_interface, jnt_pos_interface)
{
    // Do nothing for now
}

int rover_trinamic_stepper::init(struct properties props){
    this->props = props;
}

int rover_trinamic_stepper::receive()
{
    struct can_frame frame;

    frame.can_id = 1;
    frame.can_dlc = 8;
    frame.data[0] = props.receive_id;
    frame.data[1] = 0x01;   // actual position
    frame.data[2] = props.bank;
    frame.data[3] = 0x00;
    frame.data[4] = 0x00;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    ::write(can_socket, &frame, sizeof(struct can_frame));

    /* Read a message back from the CAN bus */
    recv(can_socket, &frame, sizeof(frame), NULL);

    pos = ((double)ntohl(*(uint32_t *)(frame.data + 3))) / (props.microsteps * props.steps) * 2 * M_PI;
    return 0;
}

int rover_trinamic_stepper::send()
{
    struct can_frame frame;

    double new_cmd = cmd / (2 * M_PI) * (props.microsteps * props.steps);
    
    uint32_t tmp = new_cmd;
    tmp = htonl(tmp);

    frame.can_id = 1;
    frame.can_dlc = 8;
    frame.data[0] = props.send_id;
    frame.data[1] = MVP_ABS;
    frame.data[2] = props.bank;
    *(uint32_t*)&frame.data[3] = tmp;
    ::write(can_socket, &frame, sizeof(struct can_frame));

    recv(can_socket, &frame, sizeof(frame), NULL);

    return 0;
}