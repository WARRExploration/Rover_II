#include "ros/ros.h"
#include <exploration_rover_i/motor_ror.h>
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

int main(int argc, char **argv)
{
    memset(values, 0, sizeof(values));

    ros::init(argc, argv, "controller_simple");
    ros::NodeHandle nh;

    signal(SIGINT, quit);

    pub = nh.advertise<exploration_rover_i::motor_ror>("motor_driver", 20);
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
        printf("% 4d\t% 4d\t% 4d\t% 4d\n", values[3], values[2], values[1], values[0]);

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
                values[i] += 100;
            break;
        case 'a':
            for (int i = 0; i < 4; i++)
                values[i] = 0;
            break;
        case 'y':
            for (int i = 0; i < 4; i++)
                values[i] -= 100;
            break;

        case 'w':
            values[3] += 100;
            break;
        case 's':
            values[3] = 0;
            break;
        case 'x':
            values[3] -= 100;
            break;

        case 'e':
            values[2] += 100;
            break;
        case 'd':
            values[2] = 0;
            break;
        case 'c':
            values[2] -= 100;
            break;

        case 'r':
            values[1] += 100;
            break;
        case 'f':
            values[1] = 0;
            break;
        case 'v':
            values[1] -= 100;
            break;

        case 't':
            values[0] += 100;
            break;
        case 'g':
            values[0] = 0;
            break;
        case 'b':
            values[0] -= 100;
            break;
        }

        for (int i = 0; i < 4; i++)
        {
            exploration_rover_i::motor_ror msg;
            msg.address = i + 1;
            msg.value = (i > 1) ? -values[i] : values[i];

            pub.publish(msg);
        }
    }

    return;
}