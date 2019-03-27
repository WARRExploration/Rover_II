#include <rover_control.hpp>
#include "controller_manager/controller_manager.h"
#include "hardware_interface/actuator_state_interface.h"
#include <ros/callback_queue.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_iface_node");
    ros::NodeHandle nh;
    ros::CallbackQueue queue;
    nh.setCallbackQueue(&queue);

    const char* joints[] = {"front_left_wheel_joint", "back_left_wheel_joint", "front_right_wheel_joint", "back_right_wheel_joint"};
    unsigned int can_ids[] = {0, 1, 2, 3};
    rover_interface ri (joints);
    ri.initCAN("slcan1", can_ids);

    controller_manager::ControllerManager cm(&ri,nh);

    ros::AsyncSpinner spinner(4, &queue);
    spinner.start();

    ros::Time ts = ros::Time::now();

    ros::Rate rate(10);
    while (ros::ok())
    {
         ros::Duration d = ros::Time::now() - ts;
         ts = ros::Time::now();
         ri.setSpeed();
         cm.update(ts, d);
         rate.sleep();
    }

    spinner.stop();

    return 0;
}
