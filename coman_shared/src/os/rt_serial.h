#ifndef __RT_SERIAL_H__
#define __RT_SERIAL_H__

#include <rtdm/rtserial.h>

class RTSerial {
public:
    RTSerial(const char * ,int ,int64_t , int64_t);
    virtual ~RTSerial();

    int  open();
    void close();
    int  flush();
    int  read(void *, size_t, rtser_event_t *rx_evt=NULL);
    int  write(const void *, size_t );
    
private:
    char *          ser_name;
    int             fd_ser;
    rtser_config_t  config;
    rtser_event_t   rx_event;
};


#endif

