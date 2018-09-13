#include "ros/ros.h"
#include <motor_driver.h>
#include <exploration_rover_i/tcmc.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"

ros::Publisher pub;

int values[4];

int kfd = 0;
struct termios cooked, raw;

void keyLoop();

void quit(int sig)
{
    tcsetattr(kfd, TCSANOW, &cooked);
    ros::shutdown();
    exit(0);
}

void set_motor(int address, int value)
{
    values[address - 1] = value;

    exploration_rover_i::tcmc msg;
    msg.address = address;
    msg.command = TMCL_ROR;
    msg.type = 0;
    msg.motor = 0;
    msg.value = (address > 2) ? -values[address - 1] : values[address - 1];

    pub.publish(msg);
}

int main(int argc, char **argv)
{
    memset(values, 0, sizeof(values));

    ros::init(argc, argv, "controller_simple");
    ros::NodeHandle nh;

    signal(SIGINT, quit);

    pub = nh.advertise<exploration_rover_i::tcmc>("motor_driver", 20);
    boost::thread my_thread(keyLoop);

    ros::spin();

    my_thread.interrupt();
    my_thread.join();

    return (0);
}

void keyLoop()
{
    char c;

    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &= ~(ICANON | ECHO);
    // Setting a new line, then end of file
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);

    puts("Reading from keyboard");
    puts("---------------------------");
    puts("Use arrow keys to move ROVER I.");

    while (ros::ok())
    {
        printf("\r% 6d\t% 6d\t% 6d\t% 6d\t\t", values[3], values[2], values[1], values[0]);
        fflush(stdout);

        // get the next event from the keyboard
        if (read(kfd, &c, 1) < 0)
        {
            perror("read():");
            exit(-1);
        }

        switch (c)
        {
        case 'q':
            for (int i = 0; i < 4; i++)
                set_motor(i + 1, values[i] + 100);
            break;
        case 'a':
            for (int i = 0; i < 4; i++)
                set_motor(i + 1, 0);
            break;
        case 'y':
            for (int i = 0; i < 4; i++)
                set_motor(i + 1, values[i] - 100);
            break;

        case 'w':
            set_motor(4, values[3] + 100);
            break;
        case 's':
            set_motor(4, 0);
            break;
        case 'x':
            set_motor(4, values[3] - 100);
            break;

        case 'e':
            set_motor(3, values[2] + 100);
            break;
        case 'd':
            set_motor(3, 0);
            break;
        case 'c':
            set_motor(3, values[2] - 100);
            break;

        case 'r':
            set_motor(2, values[1] + 100);
            break;
        case 'f':
            set_motor(2, 0);
            break;
        case 'v':
            set_motor(2, values[1] - 100);
            break;

        case 't':
            set_motor(1, values[0] + 100);
            break;
        case 'g':
            set_motor(1, 0);
            break;
        case 'b':
            set_motor(1, values[0] - 100);
            break;
        }
    }

    return;
}