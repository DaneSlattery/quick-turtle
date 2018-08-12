// Serial Interface
//

#ifndef SERIAL_COMM_H
#define SERIAL_COMM_H

#define DEBUG_SERIAL 0

#include <stdio.h>
#include <termios.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <map>

class SerialComm 
{
private: 
    // file descriptor 
    int fileDescriptor; // file descriptor 

    // commands and responses
    std::map<std::string, char const *> commandReplyMap 
    {
        {"AT0\r\n", "OK\r\n"},
        {"AT1\r\n", "SP\r\n"},
        {"AT2\r\n", "AT\r\n"}
    };

    int send_command(std::string command);

public:
    SerialComm();

    // Initialise the Arduino and file descriptor
    // Returns 0 if the micro responded.
    // Returns 1 if the SerialPort doesn't connect.
    // Returns 2 if the arduino responds incorrectly
    int serial_init();

    // Spin the pole right round right round.
    int stepper_spin();
    // Stop the stepper.
    int stepper_disable();

    // end the connection
    int serial_end();
};

#endif
