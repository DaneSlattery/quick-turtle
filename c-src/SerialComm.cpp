// Serial Interface Code

#include "SerialComm.h"

// globally scoped file descriptor for the serial connection
int fd;

int SerialInit()
{
//	int fd;
	char inputBuff[64] = "Empty Buffer";
	struct termios toptions;
	
	printf("Connecting to Arduino...\n");
	// Open Serial Port
	fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);
	if (fd > -1)
	{
		printf("Connected on Port %i.\n", fd);
	}
	else
	{
		printf("ERR: Not connected, Arduino probably unplugged. Stopping.\n");
		return 1;
	}

	if (fd > -1)
	{
		// Wait for Arduino reboot:
		usleep(3500000);

		// Serial Port Settings 
		tcgetattr(fd, &toptions);
		cfsetispeed(&toptions, B9600);
		cfsetospeed(&toptions, B9600);
		toptions.c_cflag &= ~PARENB;
		toptions.c_cflag &= ~CSTOPB;
		toptions.c_cflag &= ~CSIZE;
		toptions.c_cflag |= CS8;
		toptions.c_lflag |= ICANON;
		tcsetattr(fd, TCSANOW, &toptions);

		if (DEBUG_SERIAL) printf("Sending INIT command: AT0.\n");
		write(fd, "AT0\r\n", 7);

		if (DEBUG_SERIAL) printf("Waiting for response...\n");
		int numRecBytes = read(fd, inputBuff, 64);
		// insert terminating zero in the string
		inputBuff[numRecBytes] = 0;
		
		// expect OK\r\n in response
		
			
		if (inputBuff[0] == 'O'   &&
		    inputBuff[1] == 'K'   &&
		    inputBuff[2] == '\r'  &&
	            inputBuff[3] == '\n' )	
		{
			if (DEBUG_SERIAL) printf("Received %i bytes, which read: %s\n", numRecBytes, inputBuff);
			return 0;
		}
		return 2;
	}
}

int StepperSpin()
{
	char inputBuff[64] = "Empty Buffer";
	if (DEBUG_SERIAL) printf("Sending Spin command: AT1.\n");
	write(fd, "AT1\r\n", 7);

	if (DEBUG_SERIAL) printf("Waiting for response...\n");
	int numRecBytes = read(fd, inputBuff, 64);
	// insert terminating zero in the string
	inputBuff[numRecBytes] = 0;
		
	// expect OK\r\n in response
		
			
	if (inputBuff[0] == 'S'   &&
	    inputBuff[1] == 'P'   &&
	    inputBuff[2] == '\r'  &&
            inputBuff[3] == '\n' )	
	{
		if (DEBUG_SERIAL) printf("Received %i bytes, which read: %s\n", numRecBytes, inputBuff);
		return 0;
	}	
}

// Disable the stepper
int StepperDisable()
{
	char inputBuff[64] = "Empty Buffer";
	if (DEBUG_SERIAL) printf("Sending Disable command: AT2.\n");
	write(fd, "AT2\r\n", 7);

	if (DEBUG_SERIAL) printf("Waiting for response...\n");
	int numRecBytes = read(fd, inputBuff, 64);
	// insert terminating zero in the string
	inputBuff[numRecBytes] = 0;
		
	// expect OK\r\n in response
		
			
	if (inputBuff[0] == 'S'   &&
	    inputBuff[1] == 'T'   &&
	    inputBuff[2] == '\r'  &&
            inputBuff[3] == '\n' )	
	{
		if (DEBUG_SERIAL) printf("Received %i bytes, which read: %s\n", numRecBytes, inputBuff);
		return 0;
	}	
}

// end the connection
int SerialEnd()
{
	// stop the stepper if it's active
	StepperDisable();
	
	if (fd != -1)
	{
		close(fd);
		printf("Serial Connection closed.\n");
		return 0;
	}
	printf("Unable to close serial connection.\n");
	return -1;
}
