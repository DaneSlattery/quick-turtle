#include "SerialComm.h"
#include <iostream>

int main(int argc, char *argv[])
{
	int serial_rc = SerialInit();
	if (serial_rc == 0)
	{
		std::cout << "Guccinit." << std::endl;
	}
	
	for (int i = 0; i < 24; i++)
	{
        std::cout << i+1 << "/24" << std::endl;
		StepperSpin();
		usleep(15000);
	}
	return 0;
}    
