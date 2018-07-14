#include <motor_driver.h>
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <vector>
#include <string>

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

int main (int argc, char** argv){
	
	UCHAR address = 0;
	std::string dev[] = {"","","",""};
	motor_driver[3] m;
	std::ifstream setupFile ("setup.txt");
	std::string line;
	std::vector<int> type;
	std::vector<int> value;
	
	UCHAR responseAddress;
	UCHAR responseStatus;
	INT responseValue;

	INT i, j;
	
	if (argc == 1)
	{
		address = static_cast<UCHAR> argv[1]);
	}
	
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
			if (line.at(0) != '#')
			{
				//std::cout << line.substr(0,line.find_first_of(',')) << "\t";
				//std::cout << line.substr(line.find_first_of(',')+1, line.size()) << std::endl;
				type.push_back(std::stoi(line.substr(0,line.find_first_of(','))));
				value.push_back(std::stoi(line.substr(line.find_first_of(',')+1, line.size())));
			}
		}
		setupFile.close();
	}


	switch (address)
	{
		case MODULE_1:
			dev[0] = DEV_1;
			break;
		case MODULE_2:
			dev[0] = DEV_2;
			break;
		case MODULE_3:
			dev[0] = DEV_3;
			break;
		case MODULE_4:
			dev[0] = DEV_4;
			break;
		default:
			dev[0] = DEV_1;
			dev[1] = DEV_2;
			dev[2] = DEV_3;
			dev[3] = DEV_4;
	}
	

	i = 0;
	while (dev[i] != "")
	{
		printf("Module %d\n", i+1);
		m[i].init(dev, BAUDRATE);
		
		for (j=0; j<type.size(); j++)
		{
			// SAP
			m[i].SendCmd(address, TMCL_SAP, type[j], 0, value[j]);
			m[i].GetResult(&responseAddress, &responseStatus, &responseValue);
			printf("Address: %d, Status: %d, Value %d\n", responseAddress, responseStatus, responseValue);

			// STAP
			m[i].SendCmd(address, TMCL_STAP, type[j], 0, 0);
			m[i].GetResult(&responseAddress, &responseStatus, &responseValue);
			printf("Address: %d, Status: %d, Value %d\n", responseAddress, responseStatus, responseValue);
		}
		
		i++;
	}
	
	return 0;
}
