#include <iostream>
#include <cstdlib>
#include <math.h>
#include <unistd.h>
#include <rover_control_arm.hpp>

int main(int argc, char** argv)
{
    /*
     * Initializing ROS and ROS node handler
     */
    std::string param;
    ros::init(argc, argv, "test_iface_node");
    ros::NodeHandle nh;

    ros::CallbackQueue queue;
    nh.setCallbackQueue(&queue);

    std::string ifaceCAN = "slcan0";

    ROS_INFO_STREAM("CAN interface: " << ifaceCAN);
    rover_interface ri(ifaceCAN);

    /*
     * Declare controller_manager responsible for all controllers that drive motors
     */
    controller_manager::ControllerManager cm(&ri, nh);

    ros::AsyncSpinner spinner(4, &queue);
    spinner.start();

    ros::Time ts = ros::Time::now();

    ros::Rate rate(10);
    while (ros::ok())
    {
        ros::Duration d = ros::Time::now() - ts;
        ts = ros::Time::now();

        const int res = ri.canUpdate();
        /*
         * Update controller manager values
         */
        cm.update(ts, d);
        const int pub_res = ri.canPublish();
        
        rate.sleep();
    }

    spinner.stop();

    return 0;
}
