#include <motor_driver.h>
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <vector>
#include <string>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

#define BAUDRATE 9600
#define DEV_1 "/dev/ttyUSB0"
#define DEV_2 "/dev/ttyUSB1"
#define DEV_3 "/dev/ttyUSB2"
#define DEV_4 "/dev/ttyUSB3"
#define MODULE_1 1
#define MODULE_2 2
#define MODULE_3 3
#define MODULE_4 4

/*****
MAIN: To set parameter values permanently, two commands have to be sent to the motor drivers.
      First, a SAP command where a parameter is stored in SRAM.
      Second, a STAMP command where the parameter previously written by SAP is stored in EEPROM.
Parameters:
      1. Module address (1-4)
      2. Parameter to be set
      3. Value for parameter to be set
*****/

INT getParameter(std::string input, INT position)
{
	std::vector<std::string> strs;
	boost::split(strs, input, boost::is_any_of(","));
	return boost::lexical_cast<int>(strs.at(position));

}

int main (int argc, char** argv){

	UCHAR serialAddress = 0;
	std::string deviceAddress;
	std::string dev[] = {"","","",""};
	motor_driver motor;
	std::string line;
	std::vector<int> type;
	std::vector<int> value;
	std::vector<int> command;

	UCHAR responseAddress;
	UCHAR responseStatus;
	INT responseValue;

	INT i, j;

	if (argc != 3)
	{
		std::cerr << "ERROR: The program needs exactly three input arguments: \n\t1. Device address (e.g. /dev/ttyUSB1)\n\t2. Serial address for RS485\n\t3. File name to read" << std::endl;
		return -1;
	}

	deviceAddress = argv[1];
	serialAddress = boost::lexical_cast<int>(argv[2]);
	std::ifstream setupFile (argv[3]);


	/* Read lines of setup file and store data in "type" and "value" arrays
	   First number defines type of axis parameter, second number is the value to be written
	   Example:
	   # Max current
	   6,5000
	*/
	if (setupFile.is_open())
	{
		i = 0;
		while (getline(setupFile,line))
		{
			// Ignore all lines that start with '#'
			if (line.at(0) != '#' || line.at(0) != ' ')
			{
				//std::cout << line.substr(0,line.find_first_of(',')) << "\t";
				//std::cout << line.substr(line.find_first_of(',')+1, line.size()) << std::endl;
				//type.push_back(std::stoi(line.substr(0,line.find_first_of(','))));
				//value.push_back(std::stoi(line.substr(line.find_first_of(',')+1, line.size())));
				command.push_back(getParameter(line, 0));
				type.push_back(getParameter(line, 1));
				value.push_back(getParameter(line, 2));
			}
		}
		setupFile.close();
	}

	try
	{
		motor.init(deviceAddress, BAUDRATE);
	}
	catch (serial::IOException &e)
	{
		std::cerr << "Unhandled Exception: " << e.what() << std::endl;
	}

	for (j=0; j<type.size(); j++)
	{
		motor.SendCmd(serialAddress, command[j], type[j], 0, value[j]);
		motor.GetResult(&responseAddress, &responseStatus, &responseValue);
		printf("Address: %d, Status: %d, Value %d\n", responseAddress, responseStatus, responseValue);

	}
	
	return 0;
}
