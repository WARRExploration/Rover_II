#include <motor_driver.h>

// Initialize motor driver and serial connection
UCHAR motor_driver::init(std::string device, INT baudrate)
{
	try
	{
		ser.setPort(device);
		ser.setBaudrate(baudrate);
		serial::Timeout to = serial::Timeout::simpleTimeout(1000);
		ser.setTimeout(to);
		ser.open();
    	}
	catch (serial::IOException& e)
    	{
        	std::cerr << "Unable to open port" << std::endl;
        	return -1;
    	}

	if(ser.isOpen())
	{
        	std::cout << "Serial Port initialized" << std::endl;
    	}
	else
	{
        	return -1;
    	}
}

//Send a command
void motor_driver::SendCmd(UCHAR Address, UCHAR Command, UCHAR Type, UCHAR Motor, INT Value)
{
	if (ser.isOpen() == FALSE)
	{
		return;
	}

	UCHAR TxBuffer[9];
	UCHAR i;
	std::size_t bytesWritten;

	TxBuffer[0]=Address;
	TxBuffer[1]=Command;
	TxBuffer[2]=Type;
	TxBuffer[3]=Motor;
	TxBuffer[4]=Value >> 24;
	TxBuffer[5]=Value >> 16;
	TxBuffer[6]=Value >> 8;
	TxBuffer[7]=Value & 0xff;
	TxBuffer[8]=0;
	for(i=0; i<8; i++)
		TxBuffer[8]+=TxBuffer[i];

	//Now, send the 9 bytes stored in TxBuffer to the module
  	//(this is MCU specific)
	ser.write(TxBuffer, 9);
}

//Get the result
//Return TRUE when checksum of the result if okay, else return FALSE
// The follwing values are returned:
//      *Address: Host address
//      *Status: Status of the module (100 means okay)
//      *Value: Value read back by the command
UCHAR motor_driver::GetResult(UCHAR *Address, UCHAR *Status, INT *Value)
{
	if (ser.isOpen() == FALSE)
	{
		return FALSE;
	}

	UCHAR RxBuffer[9], Checksum;
	//DWORD Errors, BytesRead;
	// COMSTAT ComStat;
	int i;

  	//First, get 9 bytes from the module and store them in RxBuffer[0..8]
  	//(this is MCU specific)
	ser.read(RxBuffer, 9);

	//Check the checksum
	Checksum=0;
	for(i=0; i<8; i++)
		Checksum+=RxBuffer[i];

	if(Checksum!=RxBuffer[8]) return FALSE;

	//Decode the datagram
	*Address=RxBuffer[0];
	*Status=RxBuffer[2];
	*Value=(RxBuffer[4] << 24) | (RxBuffer[5] << 16) | (RxBuffer[6] << 8) | RxBuffer[7];

	return TRUE;
}

motor_driver::~motor_driver()
{
	ser.close();
}
