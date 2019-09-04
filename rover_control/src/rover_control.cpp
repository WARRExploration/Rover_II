#include "rover_control.hpp"

using uavcan::Array;
using uavcan::ArrayModeDynamic;
using uavcan::ArrayModeStatic;
using uavcan::IntegerSpec;
using uavcan::FloatSpec;
using uavcan::SignednessSigned;
using uavcan::SignednessUnsigned;
using uavcan::CastModeSaturate;
using uavcan::CastModeTruncate;


rover_interface::rover_interface(std::string* joint_names, std::string ifaceCAN, int idCAN)
{
    /*
     * UAVCAN Node initialization.
     * Node ID and name are required; otherwise, the node will refuse to start.
     * Version info is optional.
     */
    can_interface = ifaceCAN;
    auto& node = get_uavcan_node(ifaceCAN);
    own_id = idCAN;
    node.setNodeID(own_id);
    node.setName("org.uavcan.test");

    static uavcan::Subscriber<uavcan::equipment::esc::Status> uav_can_sub(node);
    esc_status_sub = &uav_can_sub;

    static uavcan::Publisher<uavcan::equipment::esc::RPMCommand> uav_can_pub(node);
    kv_pub = &uav_can_pub;

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
    const int kv_pub_init_res = kv_pub->init();
    if (kv_pub_init_res < 0)
    {
        ROS_ERROR("Failed to start the publisher; error: %d", kv_pub_init_res);
    }

    /*
     * TX timeout can be overridden if needed.
     * Default value should be OK for most use cases.
     */
    kv_pub->setTxTimeout(uavcan::MonotonicDuration::fromMSec(1000));

    /*
     * Priority of outgoing tranfers can be changed as follows.
     * Default priority is 16 (medium).
     */
    kv_pub->setPriority(uavcan::TransferPriority::MiddleLower);

    /*
     * Subscribing to status messages log messages of type uavcan.protocol.debug.LogMessage.
     */

     const int esc_status_sub_res = esc_status_sub->start(
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
    ROS_INFO_STREAM("UAVCAN node initialized, Node ID: " << static_cast<int>(node.getNodeID().get()));

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

int rover_interface::canUpdate()
{
    //const int res = node.spin(uavcan::MonotonicDuration::fromMSec(1000));
    auto& node = get_uavcan_node(can_interface);
    const int res = node.spinOnce();
    if (res < 0)
    {
        ROS_ERROR("Transient failure: %d", res);
    }
    return res;
}

int rover_interface::canPublish()
{
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
    ROS_INFO("Command value [0]: %f", cmd[0]);
    test.push_back(floor(5*100*cmd[0]*60/2/M_PI));
    test.push_back(floor(5*100*cmd[1]*60/2/M_PI));
    test.push_back(floor(5*100*cmd[2]*60/2/M_PI));
    test.push_back(floor(5*100*cmd[3]*60/2/M_PI));
    kv_msg.rpm = test;

    /*
     * Publishing the message.
     */
    const int pub_res = kv_pub->broadcast(kv_msg);
    if (pub_res < 0)
    {
        ROS_ERROR("KV publication failure: %d", pub_res);
    }
    return pub_res;
}

Node& rover_interface::get_uavcan_node(std::string ifaceCAN)
{
    /**
     * Node object will be constructed at the time of the first access.
     * Note that most library objects are noncopyable (e.g. publishers, subscribers, servers, callers, timers, ...).
     * Attempt to copy a noncopyable object causes compilation failure.
     */
    static Node node(getCanDriver(ifaceCAN), getSystemClock());
    return node;
}
