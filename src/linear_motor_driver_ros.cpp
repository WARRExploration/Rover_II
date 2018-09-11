#include <linear_motor_driver.h>
#include "ros/ros.h"
//#include <exploration_rover_i/linear_motor_setpos.h>
//#include <exploration_rover_i/linear_motor_info.h>
#include "std_msgs/Int16.h"
#include <string>
#include <vector>


linear_motor_driver linear_driver;

void handleInput(const std_msgs::Int16& msg)
{
	linear_driver.SendCmd(SET_POSITION, msg.data);
	ROS_INFO("Linear_Motor: %d", (int)msg.data);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "linear_motor_driver");

	ros::NodeHandle nh;
	ros::NodeHandle nhp("~");


	// get device
	std::string device;
	if (!nhp.getParam("device", device))
	{
		ROS_ERROR("Device not set!");
		exit(1);
	}

	// init device
	linear_driver.init(device, 57600);

	
	// start the subscriber
	ros::Subscriber input = nh.subscribe("linear_motor_driver", 10, handleInput);
	//ros::Publisher pub = nh.advertise<exploration_rover_i::linear_motor_info>("linear_motor_info", 1000);

	// lifecycle
	//ros::Rate loop_rate(info_framerate);

	//while (ros::ok())
	//{
		/*for (int i = 0; i < infos.size(); i++)
		{
			for (int j = 0; j < info_addresses.size(); j++)
			{*/
				//int address = info_addresses[j];
				//int motor = 0;
				//int type = infos[i];

				//if (mode == MODE_USB && address != usb_address)
				//	continue;

				//UCHAR r_control;// status;
				//INT value;
				//driver.SendCmd(GetFeedback, 0);
				//driver.GetResult(&r_control, &value);
				// ROS_INFO("Address: %x\t type: %x\t value: %d", usb_address, infos[i], value);

				// an error occured
				/*if (status != 100)
				{
					ROS_ERROR("An error occured when trying to get info from motor %x (status: %x)", address, status);
					continue;
				}*/

				//exploration_rover_i::linear_motor_info info_msg;
				//info_msg.address = address;
				//info_msg.motor = motor;
				//info_msg.type = type;
				//info_msg.value = value;

				//pub.publish(info_msg);
			/*}
		}*/

		//ros::spinOnce();
		//loop_rate.sleep();
	//}
	ros::spin();
	return 0;
}

