// Serial Interface
//

#ifndef SERIAL_COMM_H
#define SERIAL_COMM_H

#define DEBUG 0

#include <stdio.h>
#include <termios.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <stdint.h>
#include <sys/ioctl.h>

// Initialise the Arduino and file descriptor
// Returns 0 if the micro responded.
// Returns 1 if the SerialPort doesn't connect.
// Returns 2 if the arduino responds incorrectly
int SerialInit();

// Spin the pole right round right round.
int StepperSpin();

// Stop the stepper.
int StepperStop();

// end the connection
int SerialEnd();


#endif
