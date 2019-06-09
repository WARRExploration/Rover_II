#include <iostream>
#include <cstdlib>
#include <math.h>
#include <unistd.h>
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

#include <rover_control.hpp>
#include "controller_manager/controller_manager.h"
#include "hardware_interface/actuator_state_interface.h"
#include <ros/callback_queue.h>



using uavcan::Array;
using uavcan::ArrayModeDynamic;
using uavcan::ArrayModeStatic;
using uavcan::IntegerSpec;
using uavcan::FloatSpec;
using uavcan::SignednessSigned;
using uavcan::SignednessUnsigned;
using uavcan::CastModeSaturate;
using uavcan::CastModeTruncate;

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
            ROS_ERROR("Failed to add iface");
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
    const int self_node_id = 123;
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
     * Dependent objects (e.g. publishers, subscribers, servers, callers, timers, ...) can be initialized only
     * if the node is running. Note that all dependent objects always keep a reference to the node object.
     */

    const int node_start_res = node.start();
    if (node_start_res < 0)
    {
        ROS_ERROR("Failed to start the node; error:%d", node_start_res);
    }

    /*
     * Create the publisher object to broadcast standard key-value messages of type uavcan.protocol.debug.KeyValue.
     * Keep in mind that most classes defined in the library are not copyable; attempt to copy objects of
     * such classes will result in compilation failure.
     * A publishing node won't see its own messages.
     */
    uavcan::Publisher<uavcan::equipment::esc::RPMCommand> kv_pub(node);
    const int kv_pub_init_res = kv_pub.init();
    if (kv_pub_init_res < 0)
    {
        ROS_ERROR("Failed to start the publisher; error: %d", kv_pub_init_res);
    }

    /*
     * TX timeout can be overridden if needed.
     * Default value should be OK for most use cases.
     */
    kv_pub.setTxTimeout(uavcan::MonotonicDuration::fromMSec(1000));

    /*
     * Priority of outgoing tranfers can be changed as follows.
     * Default priority is 16 (medium).
     */
    kv_pub.setPriority(uavcan::TransferPriority::MiddleLower);

    /*
     * Subscribing to status messages log messages of type uavcan.protocol.debug.LogMessage.
     */
     uavcan::Subscriber<uavcan::equipment::esc::Status> esc_status_sub(node);
     const int esc_status_sub_res = esc_status_sub.start(
         [&](const uavcan::ReceivedDataStructure<uavcan::equipment::esc::Status>& msg)
         {
            /*
             * The message will be streamed in YAML format.
             */
            std::cout << msg << std::endl;
            /*
             * If the standard iostreams are not available (they rarely available in embedded environments),
             * use the helper class uavcan::OStream defined in the header file <uavcan/helpers/ostream.hpp>.
             */
            // uavcan::OStream::instance() << msg << uavcan::OStream::endl;
        }
    );
    if (esc_status_sub_res < 0)
    {
        throw std::runtime_error("Failed to start the subscriber; error: " + std::to_string(esc_status_sub_res));
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

    /*
     * Initializing ROS and ROS node handler
     */
    std::string param;
    ros::init(argc, argv, "test_iface_node");
    ros::NodeHandle nh;

    ros::CallbackQueue queue;
    nh.setCallbackQueue(&queue);

    const char* joints[] = {"front_left_wheel_joint", "back_left_wheel_joint", "front_right_wheel_joint", "back_right_wheel_joint"};
    rover_interface ri (joints);

    /*
     * Declare controller_manager responsible for all controllers that drive motors
     */
    controller_manager::ControllerManager cm(&ri,nh);

    /*
     * Declare a publisher which publishes
     */
    //unsigned int status;

    ros::AsyncSpinner spinner(4, &queue);
    spinner.start();

    ros::Time ts = ros::Time::now();

    ros::Rate rate(10);
    while (ros::ok())
    {
        ros::Duration d = ros::Time::now() - ts;
        ts = ros::Time::now();

        //const int res = node.spin(uavcan::MonotonicDuration::fromMSec(1000));
        const int res = node.spinOnce();
        if (res < 0)
        {
            ROS_ERROR("Transient failure: %d", res);
        }

        /*
         * Publishing a random value using the publisher created above.
         * All message types have zero-initializing default constructors.
         * Relevant usage info for every data type is provided in its DSDL definition.
         */
        uavcan::equipment::esc::RPMCommand kv_msg;  // Always zero initialized
        //uavcan::primitive::array test[2] = {std::rand(), std::rand()};
        Array<IntegerSpec<18, SignednessSigned, CastModeSaturate>, ArrayModeDynamic, 20> test;
        // velocity (cmd) is in rad/s -> rpm = cmd * 60 / (2 pi)
        // multiply * 100 due to gear box
        test.push_back(floor(5*100*ri.cmd[0]*60/2/M_PI));
        test.push_back(floor(5*100*ri.cmd[1]*60/2/M_PI));
        test.push_back(floor(5*100*ri.cmd[2]*60/2/M_PI));
        test.push_back(floor(5*100*ri.cmd[3]*60/2/M_PI));
        kv_msg.rpm = test;

        /*
         * Publishing the message.
         */
        const int pub_res = kv_pub.broadcast(kv_msg);
        if (pub_res < 0)
        {
            ROS_ERROR("KV publication failure: %d", pub_res);
        }

        //ROS_INFO("Cmd value: %d", kv_msg.rpm[1]);

        /*
         * Update controller manager values
         */
        cm.update(ts, d);
        rate.sleep();
    }

    spinner.stop();

    return 0;
}
