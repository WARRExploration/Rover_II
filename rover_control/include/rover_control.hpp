// Standard Libs
#include <string>

// ROS Libs
#include <ros/ros.h>

// ROS HW interface libs
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <uavcan/uavcan.hpp>
#include <uavcan_linux/uavcan_linux.hpp>
#include <uavcan/marshal/types.hpp>

/*
 * We're going to use messages of type uavcan.equipment.esc.RPMCommand, so the appropriate header must be included.
 * Given a data type named X, the header file name would be:
 *      X.replace('.', '/') + ".hpp"
 */
#include <uavcan/equipment/esc/RPMCommand.hpp> // uavcan.equipment.esc.RPMCommand
#include <uavcan/equipment/esc/Status.hpp> // uavcan.equipment.esc.RPMCommand

#include "controller_manager/controller_manager.h"
#include "hardware_interface/actuator_state_interface.h"
#include <ros/callback_queue.h>

/**
 * These functions are platform dependent, so they are not included in this example.
 * Refer to the relevant platform documentation to learn how to implement them.
 */
extern uavcan::ISystemClock& getSystemClock();
extern uavcan::ICanDriver& getCanDriver(std::string ifaceName);

/**
 * Memory pool size largely depends on the number of CAN ifaces and on application's logic.
 * Please read the documentation for the class uavcan::Node to learn more.
 */
constexpr unsigned NodeMemoryPoolSize = 16384;

typedef uavcan::Node<NodeMemoryPoolSize> Node;


class rover_interface : public hardware_interface::RobotHW
{
public:
    rover_interface(std::string* joint_names, std::string ifaceCAN, int idCAN);
    int canUpdate();
    int canPublish();

private:
    int can_socket;
    unsigned int can_timeout;
    unsigned int own_id;
    unsigned int can_ids[4];
    std::string can_interface;
    static Node& get_uavcan_node(std::string ifaceCAN);
    uavcan::Subscriber<uavcan::equipment::esc::Status>* esc_status_sub;
    uavcan::Publisher<uavcan::equipment::esc::RPMCommand>* kv_pub;

    hardware_interface::JointStateInterface jnt_state_interface;
    //hardware_interface::PositionJointInterface jnt_pos_interface;
    hardware_interface::VelocityJointInterface jnt_vel_interface;

    double cmd[4];
    double vel[4];
    double prev_cmd[4];
    double pos[4];
    double eff[4];
};
