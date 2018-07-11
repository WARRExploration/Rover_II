//#include <ros/ros.h>
//#include <serial/serial.h>
#include <motor_driver.h>
#include <iostream>
#include <stdlib.h>
#include <sstream>

int main(int argc, char **argv)
{

	std::string dev = "/dev/ttyACM";
	UCHAR address = 3;		  // address of motor driver
	UCHAR command = TMCL_ROR; // rotate right
	UCHAR type = 0;			  // don't care
	UCHAR motor = 0;
	INT value = 0;
	if (argc > 1)
		value = atoi(argv[1]);

	UCHAR responseAddress;
	UCHAR responseStatus;
	INT responseValue;

	motor_driver m[4];
	int w[4];
	w[0] = 1;
	w[1] = 1;
	w[2] = -1;
	w[3] = -1;

	for (int i = 0; i < 4; i++)
	{
		std::stringstream ss;
		ss << dev << i;
		m[i].init(ss.str(), 9600);

		m[i].SendCmd(1, command, type, motor, w[i] * value);
		m[i].GetResult(&responseAddress, &responseStatus, &responseValue);
		printf("Adress: %d, Status: %d, Value %d\n", responseAddress, responseStatus, responseValue);
	}

	return 0;
}
