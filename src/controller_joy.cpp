#include "ros/ros.h"
#include "sensor_msgs/Joy.h"

#include <exploration_rover_i/tcmc.h>
#include <motor_driver.h>

#define MAX_SPEED 3200

ros::Publisher pub;

void callback(const sensor_msgs::Joy &msg)
{
    float x = -msg.axes[0];
    float y = msg.axes[1];

    float speed = y * MAX_SPEED;
    float steering = x * MAX_SPEED;

    float max_value = abs(speed) + abs(steering);
    if(max_value > MAX_SPEED)
    {
        speed *= MAX_SPEED / max_value;
        steering *= MAX_SPEED / max_value;
    }

    float left = speed + steering;
    float right = speed - steering;

    for (int i = 0; i < 4; i++)
    {
        exploration_rover_i::tcmc motor_cmd;
        motor_cmd.address = i;
        motor_cmd.command = TMCL_ROR;
        motor_cmd.type = 0;
        motor_cmd.motor = 0;
        motor_cmd.value = (i < 2) ? -right : left;
        pub.publish(motor_cmd);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller_joy");

    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    ros::Subscriber sub = nh.subscribe("joy", 10, callback);
    pub = nh.advertise<exploration_rover_i::tcmc>("motor_driver", 20);

    ros::spin();

    return 0;
}