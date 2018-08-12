// Serial Interface Code

#include "SerialComm.h"

SerialComm::SerialComm(): fileDescriptor(-1)
{

}

int SerialComm::send_command(std::string command)
{
    char inputBuff[64] = "Empty Buffer";

	if (DEBUG_SERIAL) printf("Sending command: %s\n", command.c_str());
	write(fileDescriptor, command.c_str(), 7);

	if (DEBUG_SERIAL) printf("Waiting for response...\n");
	int numRecBytes = read(fileDescriptor, inputBuff, 64);
	// insert terminating zero in the string
	inputBuff[numRecBytes] = 0;
	
	// expect OK\r\n in response
	if (strcmp(inputBuff, commandReplyMap.at(command)) == 0) 	
	{
		if (DEBUG_SERIAL) printf("Received %i bytes, which read: %s\n", numRecBytes, inputBuff);
		return numRecBytes;
	}
	return 0;
}


int SerialComm::serial_init()
{
	char inputBuff[64] = "Empty Buffer";
	struct termios toptions;
	
	printf("Connecting to Arduino...\n");

	// Open Serial Port as a file
	fileDescriptor = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);

	if (fileDescriptor > -1)
	{
		printf("Connected on Port %i.\n", fileDescriptor);
		
		// Wait for Arduino reboot:
		usleep(3500000);

		// Serial Port Settings 
		tcgetattr(fileDescriptor, &toptions);
		cfsetispeed(&toptions, B9600);
		cfsetospeed(&toptions, B9600);
		toptions.c_cflag &= ~PARENB;
		toptions.c_cflag &= ~CSTOPB;
		toptions.c_cflag &= ~CSIZE;
		toptions.c_cflag |= CS8;
		toptions.c_lflag |= ICANON;
		tcsetattr(fileDescriptor, TCSANOW, &toptions);

		int numRecBytes = send_command("AT0\r\n");
		
		if (numRecBytes > 0) 
		{
			return 0;
		}
		else
		{
			return 2;
		}

	}
	else
	{
		printf("ERR: Not connected, Arduino probably unplugged. Stopping.\n");
		return 1;
	}

}

int SerialComm::stepper_spin()
{
	int numRecBytes = send_command("AT1\r\n");

	if (numRecBytes > 0) 
	{
		return 0;
	}
	else
	{
		return 2;
	}
}

// Disable the stepper
int SerialComm::stepper_disable()
{
	int numRecBytes = send_command("AT2\r\n");
	
	if (numRecBytes > 0) 
	{
		return 0;
	}
	else
	{
		return 2;
	}
}

// end the connection
int SerialComm::serial_end()
{
	// stop the stepper if it's active
	stepper_disable();
	
	if (fileDescriptor != -1)
	{
		close(fileDescriptor);
		printf("Serial Connection closed.\n");
		return 0;
	}
	printf("Unable to close serial connection.\n");
	return -1;
}
