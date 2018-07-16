#include <termios.h> // terminal io (serial port) interface
#include <fcntl.h>   // File control definitions
#include <errno.h>   // Error number definitions
//#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>

#include "nrt_serial.h"

// baudrate is fixed to 115200

NRTSerial::NRTSerial(const std::string name, int baud_rate):
ser_name(name) /*,baudrate(baud_rate)*/ {}

NRTSerial::~NRTSerial() {}

int NRTSerial::open() {

    int err = 0; 

    /* open serial */
    fd_ser = ::open(ser_name.c_str(), O_RDWR | O_NOCTTY );
    if (fd_ser < 0) {
        err = fd_ser;
        printf("can't open %s, %s\n", ser_name.c_str(), strerror(-err));
        return err;
    }

    //Get the current options for the port...
    struct termios options;
    tcgetattr(fd_ser, &options);

    //set the baud rate to 115200
    int baudRate = B115200;
    cfsetospeed(&options, baudRate);
    cfsetispeed(&options, baudRate);

    //set the number of data bits.
    options.c_cflag &= ~CSIZE;  // Mask the character size bits
    options.c_cflag |= CS8;

    //set the number of stop bits to 1
    options.c_cflag &= ~CSTOPB;

    //Set parity to None
    options.c_cflag &=~PARENB;

    //set for non-canonical (raw processing, no echo, etc.)
    options.c_iflag = IGNPAR; // ignore parity check close_port(int
    options.c_oflag = 0; // raw output
    options.c_lflag = 0; // raw input

    //Time-Outs -- won't work with NDELAY option in the call to open
    options.c_cc[VMIN]  = 0;   // block reading until RX x characers. If x = 0, it is non-blocking.
    options.c_cc[VTIME] = 100;   // Inter-Character Timer -- i.e. timeout= x*.1 s

    //Set local mode and enable the receiver
    options.c_cflag |= (CLOCAL | CREAD);

    tcflush(fd_ser,TCIOFLUSH);
    
      //Set the new options for the port...
    if ( tcsetattr(fd_ser, TCSANOW, &options) != 0){ //For error message
        printf("Configuring comport failed\n");
        return -1;
    }

    return fd_ser;   
}

void NRTSerial::close() {
    // return 0 on success, otherwise a negative error code.
    ::close(fd_ser);
    printf("Close %s\n", ser_name.c_str());
}

int NRTSerial::flush() {
    return tcflush(fd_ser, TCIOFLUSH);
}

int NRTSerial::read(void *buf, size_t nbyte, void * dummy) {

    return ::read(fd_ser, buf, nbyte);
}

int NRTSerial::write(const void *buf, size_t nbyte) {

    return ::write(fd_ser, buf, nbyte);
}

