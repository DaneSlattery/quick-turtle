// Dane Slattery 
// Nick Raal
// Image Capture C++ Algorithm

// http://libserial.readthedocs.io/en/latest/tutorial.html
#include <iostream>
#include <stdio.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

using namespace std;

int fd;

char *buff;

// This function initiates communication with the Arduino Nano using a serial connection 
// initMicro(string filename)


int main()
{
	// variable declaration
	float totalAngle = 0.0;

	// Initiate Camera
	
	// Initiate Microcontroller
//	microPort = initMicro("/dev/ttyUSB0");
    fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
    sleep(1);
//    struct termios toptions;
//    
//    tcgetattr(fd, &toptions);
//    
//    cfsetispeed(&toptions, B9600);
//    cfsetospeed(&toptions, B9600);
//    
//    /* 8 bits, no parity, no stop bits */
//    toptions.c_cflag &= ~PARENB;
//    toptions.c_cflag &= ~CSTOPB;
//    toptions.c_cflag &= ~CSIZE;
//    toptions.c_cflag |= CS8;
//    /* no hardware flow control */
//    toptions.c_cflag &= ~CRTSCTS;
//    /* enable receiver, ignore status lines */
//    toptions.c_cflag |= CREAD | CLOCAL;
//    /* disable input/output flow control, disable restart chars */
//    toptions.c_iflag &= ~(IXON | IXOFF | IXANY);
//    /* disable canonical input, disable echo,
//    disable visually erase chars,
//    disable terminal-generated signals */
//    toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
//    /* disable output processing */
//    toptions.c_oflag &= ~OPOST;
    
    fcntl(fd, F_SETFL, 0);
    printf("Port open.\n");
    
    write(fd, "1", 1);
    int rd = read(fd, buff, 10);
    
    printf("%d bytes received are %s \n", rd, buff);
    
    close(fd);
	//Loop
	// Capture 1st Image
	
	// Rotate platform 15deg
	
		
}
