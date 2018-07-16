#ifndef __NRT_SERIAL_H__
#define __NRT_SERIAL_H__

#include <stdlib.h>
#include <string>

class NRTSerial {
public:
    NRTSerial(const std::string ,int );
    virtual ~NRTSerial();

    int  open();
    void close();
    int  flush();
    int  read(void *, size_t, void * dummy=NULL);
    int  write(const void *, size_t );

private:
    std::string  ser_name;
    int     fd_ser;
    int     baudrate;
};



#endif

