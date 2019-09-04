#include <iostream>
#include <cstdlib>
#include <math.h>
#include <unistd.h>
#include <rover_control.hpp>

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

    std::string joints[] = {"front_left_wheel_joint", "back_left_wheel_joint", "front_right_wheel_joint", "back_right_wheel_joint"};
    int idCAN = 123;
    std::string ifaceCAN = "slcan0";

    ROS_INFO("CAN ID: %d", idCAN);
    ROS_INFO_STREAM("CAN interface: " << ifaceCAN);
    rover_interface ri(joints, ifaceCAN, idCAN);

    ROS_INFO("Rover interface object created.");

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
        const int pub_res = ri.canPublish();

        /*
         * Update controller manager values
         */
        cm.update(ts, d);
        rate.sleep();
    }

    spinner.stop();

    return 0;
}
