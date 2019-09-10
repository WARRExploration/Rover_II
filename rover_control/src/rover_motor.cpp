#include <rover_motor.hpp>

rover_motor::rover_motorrover_motor(std::string joint_name, int can_socket, 
    hardware_interface::JointStateInterface *jnt_state_interface, 
    hardware_interface::PositionJointInterface *jnt_pos_interface)
{
    this->can_socket = can_socket;

    hardware_interface::JointStateHandle state_handle(joint_name, &pos, &vel, &eff);
    jnt_state_interface->registerHandle(state_handle);

    hardware_interface::JointHandle joint_handle(state_handle, &cmd);
    jnt_pos_interface->registerHandle(joint_handle);
}