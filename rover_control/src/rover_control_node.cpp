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
    unsigned int can_ids[] = {0x001, 0x002, 0x003, 0x004};
    unsigned int own_id = 0x123;
    rover_interface ri (joints);
    if (ri.initCAN("slcan0", can_ids, own_id, 1) == 0)
    {
        ROS_INFO("SocketCAN interface established.");
    }
    else
    {
        ROS_ERROR("SocketCAN interface could not be established.");
    }

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
         cm.update(ts, d);
         rate.sleep();
    }

    spinner.stop();

    return 0;
}
