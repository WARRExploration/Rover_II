//#include <ros/ros.h>
//#include <serial/serial.h>
#include <motor_driver.h>
#include <iostream>
#include <stdlib.h>


int main (int argc, char** argv){

	std::string dev1 = "/dev/ttyUSB0";
	UCHAR address = 3; // address of motor driver
	UCHAR command = TMCL_ROR; // rotate right
	UCHAR type = 0;	// don't care
	UCHAR motor = 0;
	INT value = 0;
	if(argc > 1)
		value = atoi(argv[1]);

	UCHAR responseAddress;
	UCHAR responseStatus;
	INT responseValue;

	motor_driver m1;
	m1.init(dev1, 9600);
	m1.SendCmd(1, command, type, motor, value);
	m1.GetResult(&responseAddress, &responseStatus, &responseValue);
	printf("Adress: %d, Status: %d, Value %d\n", responseAddress, responseStatus, responseValue);
	m1.SendCmd(2, command, type, motor, value);
	m1.GetResult(&responseAddress, &responseStatus, &responseValue);
	printf("Adress: %d, Status: %d, Value %d\n", responseAddress, responseStatus, responseValue);


	return 0;
}
