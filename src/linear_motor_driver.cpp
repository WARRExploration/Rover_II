#include <linear_motor_driver.h>

// Initialize motor driver and serial connection
UCHAR linear_motor_driver::init(std::string device, INT baudrate)
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

	if (ser.isOpen())
	{
		std::cout << "Serial Port initialized" << std::endl;
	}
	else
	{
		return -1;
	}
}

//Send a command
void linear_motor_driver::SendCmd(UCHAR Control, INT Value)
{
	if (ser.isOpen() == FALSE)
	{
		return;
	}

	UCHAR TxBuffer[3];
	std::size_t bytesWritten;

	TxBuffer[0] = Control;
	TxBuffer[1] = Value;
	TxBuffer[2] = Value >> 8;


	//Now, send the 2 bytes stored in TxBuffer to the module
	//(this is MCU specific)
	ser.write(TxBuffer, 3);
}

UCHAR linear_motor_driver::GetResult(UCHAR *Control, INT *Value) // ??
{
	if (ser.isOpen() == FALSE)
	{
		return FALSE;
	}

	UCHAR RxBuffer[3];
	//DWORD Errors, BytesRead;
	// COMSTAT ComStat;

	//First, get 9 bytes from the module and store them in RxBuffer[0..8]
	//(this is MCU specific)
	ser.read(RxBuffer, 3);

	//Decode the datagram
	*Control = RxBuffer[0];
	//*Status = RxBuffer[2];
	*Value = (RxBuffer[1]) | (RxBuffer[2] << 16);

	return TRUE;
}

linear_motor_driver::~linear_motor_driver()
{
	ser.close();
}

