#include <iostream>
#include <cstdlib>
#include <unistd.h>
#include <uavcan/uavcan.hpp>
#include <uavcan_linux/uavcan_linux.hpp>

#include <rover_control.hpp>
#include "controller_manager/controller_manager.h"
#include "hardware_interface/actuator_state_interface.h"
#include <ros/callback_queue.h>

/**
 * These functions are platform dependent, so they are not included in this example.
 * Refer to the relevant platform documentation to learn how to implement them.
 */
 uavcan::ISystemClock& getSystemClock()
 {
     static uavcan_linux::SystemClock clock;
     return clock;
 }

uavcan::ICanDriver& getCanDriver()
{
    static uavcan_linux::SocketCanDriver driver(dynamic_cast<const uavcan_linux::SystemClock&>(getSystemClock()));
    if (driver.getNumIfaces() == 0)     // Will be executed once
    {
        if (driver.addIface("slcan0") < 0)
        {
            //ROS_ERROR("Failed to add iface");
        }
    }
    return driver;
}
/**
 * Memory pool size largely depends on the number of CAN ifaces and on application's logic.
 * Please read the documentation for the class uavcan::Node to learn more.
 */
constexpr unsigned NodeMemoryPoolSize = 16384;

typedef uavcan::Node<NodeMemoryPoolSize> Node;

/**
 * Node object will be constructed at the time of the first access.
 * Note that most library objects are noncopyable (e.g. publishers, subscribers, servers, callers, timers, ...).
 * Attempt to copy a noncopyable object causes compilation failure.
 */
static Node& getNode()
{
    static Node node(getCanDriver(), getSystemClock());
    return node;
}

int main(int argc, char** argv)
{
    const int self_node_id = 0x123;
    /*
     * Node initialization.
     * Node ID and name are required; otherwise, the node will refuse to start.
     * Version info is optional.
     */
    auto& node = getNode();

    node.setNodeID(self_node_id);

    node.setName("org.uavcan.test");

    uavcan::protocol::SoftwareVersion sw_version;  // Standard type uavcan.protocol.SoftwareVersion
    sw_version.major = 1;
    node.setSoftwareVersion(sw_version);

    uavcan::protocol::HardwareVersion hw_version;  // Standard type uavcan.protocol.HardwareVersion
    hw_version.major = 1;
    node.setHardwareVersion(hw_version);

    /*
     * Start the node.
     * All returnable error codes are listed in the header file uavcan/error.hpp.
     */

    const int node_start_res = node.start();
    if (node_start_res < 0)
    {
        ROS_ERROR("Failed to start the node; error:%d", node_start_res);
    }

    /*
     * Informing other nodes that we're ready to work.
     * Default mode is INITIALIZING.
     */
    node.setModeOperational();

    /*
     * Some logging.
     * Log formatting is not available in C++03 mode.
     */
     /*
    node.getLogger().setLevel(uavcan::protocol::debug::LogLevel::DEBUG);
    node.logInfo("main", "CAN Node initialized! Node ID: %*",
                 static_cast<int>(node.getNodeID().get()));
    */
    ROS_INFO("UAVCAN node initialized.");


    ros::init(argc, argv, "test_iface_node");
    ros::NodeHandle nh;
    ros::CallbackQueue queue;
    nh.setCallbackQueue(&queue);

    const char* joints[] = {"front_left_wheel_joint", "back_left_wheel_joint", "front_right_wheel_joint", "back_right_wheel_joint"};
    unsigned int can_ids[] = {0x001, 0x002, 0x003, 0x004};
    unsigned int own_id = 0x123;
    rover_interface ri (joints);
    /*
    if (ri.initCAN("slcan0", can_ids, own_id, 1) == 0)
    {
        ROS_INFO("SocketCAN interface established.");
    }
    else
    {
        ROS_ERROR("SocketCAN interface could not be established.");
    }
    */

    controller_manager::ControllerManager cm(&ri,nh);

    unsigned int status;

    ros::AsyncSpinner spinner(4, &queue);
    spinner.start();

    ros::Time ts = ros::Time::now();

    ros::Rate rate(10);
    while (ros::ok())
    {
        ros::Duration d = ros::Time::now() - ts;
        ts = ros::Time::now();

        /*
        status = ri.setSpeed();
        if(status != STATUS_OK)
        {
         ROS_INFO_STREAM("SocketCAN Status Error: " << status);
        };
        status = ri.getSpeed(can_ids[0]);
        if(status != STATUS_OK)
        {
         ROS_INFO_STREAM("SocketCAN Status Error: " << status);
        }
        status = ri.getSpeed(can_ids[1]);
        if(status != STATUS_OK)
        {
         ROS_INFO_STREAM("SocketCAN Status Error: " << status);
        }
        status = ri.getSpeed(can_ids[2]);
        if(status != STATUS_OK)
        {
         ROS_INFO_STREAM("SocketCAN Status Error: " << status);
        }
        status = ri.getSpeed(can_ids[3]);
        if(status != STATUS_OK)
        {
         ROS_INFO_STREAM("SocketCAN Status Error: " << status);
        }
        */
        /*
        * Random status transitions.
        * In real applications, the status code shall reflect node's health.
        */
        const float random = std::rand() / float(RAND_MAX);
        if (random < 0.7)
        {
            node.setHealthOk();
        }
        else if (random < 0.9)
        {
            node.setHealthWarning();
        }
        else
        {
            node.setHealthError();
        }
        //ROS_INFO("Random value: %f", random);
        cm.update(ts, d);
        rate.sleep();
    }

    spinner.stop();

    return 0;
}
