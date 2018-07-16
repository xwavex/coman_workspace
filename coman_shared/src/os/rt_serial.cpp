#include "rt_serial.h"

#include <rtdk.h>
#include <stdlib.h>
#include <string.h>

#include <utils.h>

//extern "C" uint64_t get_time_ns(void);

RTSerial::RTSerial(const char * name, int baud_rate, int64_t event_timeout_ns, int64_t rx_timeout)
{
    ser_name = strdup(name);

    config.config_mask       = 0xFFFF;
    config.baud_rate         = baud_rate;
    config.parity            = RTSER_DEF_PARITY;
    config.data_bits         = RTSER_DEF_BITS;
    config.stop_bits         = RTSER_DEF_STOPB;
    config.handshake         = RTSER_DEF_HAND;
    config.fifo_depth        = RTSER_DEF_FIFO_DEPTH;
    config.rx_timeout        = rx_timeout; // --> RTDM_TIMEOUT_INFINITE blocking -- RTDM_TIMEOUT_NONE non blocking 
    config.tx_timeout        = RTSER_DEF_TIMEOUT; // --> RTDM_TIMEOUT_INFINITE blocking
    config.event_timeout     = event_timeout_ns; 
    config.timestamp_history = RTSER_RX_TIMESTAMP_HISTORY;
    config.event_mask        = RTSER_EVENT_RXPEND;

}

RTSerial::~RTSerial() {

    free(ser_name);
}

int RTSerial::open() {

    int err = 0; 

    /* open rtserN */
    fd_ser = rt_dev_open( ser_name, 0);
    if (fd_ser < 0) {
        err = fd_ser;
        rt_printf("can't open %s, %s\n", ser_name, strerror(-err));
        return err;
    }

    /* write config */
    // return 0 on success, otherwise negative error code
    err = rt_dev_ioctl(fd_ser, RTSER_RTIOC_SET_CONFIG, &config);
    if (err) {
        rt_printf("RTSerial::open(): error in rt_dev_ioctl(): %s\n", strerror(-err));
        return err;
    }

    //rt_printf("serial %s configured\n", ser_name);

    return fd_ser;   
}

void RTSerial::close() {
    // return 0 on success, otherwise a negative error code.
    rt_dev_close(fd_ser);
    rt_printf("Close %s\n", ser_name);
}

int RTSerial::read(void *buf, size_t nbyte, rtser_event_t * rx_event_ptr) {

    ssize_t read = 0;
    int    err = 0;
    uint64_t t0, t1;

    // Wait on serial device events according to previously set mask
    // return 0 on success, otherwise negative error code
    //t0 = get_time_ns();
    err = rt_dev_ioctl(fd_ser, RTSER_RTIOC_WAIT_EVENT, &rx_event);
    if (err) {
        rt_printf("error on RTSER_RTIOC_WAIT_EVENT, %s\n", strerror(-err));
        return err; 
    }
    //t1 = get_time_ns();
    //rt_printf("rt_dev_ioctl duration %lld ns\n", t1-t0);

    if (rx_event_ptr) {
        memcpy((void*)rx_event_ptr,(void*)&rx_event, sizeof(rx_event));
    }

    // -- NOTE -- if config.rx_timeout = RTDM_TIMEOUT_INFINITE --> block until nbytes are read !!!!!
    // return number of bytes read, otherwise negative error code
    //t0 = get_time_ns();
    read = rt_dev_read(fd_ser, buf, nbyte);
    //t1 = get_time_ns();
    //rt_printf("rt_dev_read duration %lld ns\n", t1-t0);

    if (read < 0) {
        rt_printf("error on rt_dev_read, %s\n", strerror(-read));
    } else if (static_cast<size_t>(read) != nbyte) {
        rt_printf("only %d / %d byte received\n", read, nbyte);
    }
    return read;
}

int RTSerial::write(const void *buf, size_t nbyte) {

    // return number of bytes written, otherwise negative error code
    ssize_t written = rt_dev_write(fd_ser, buf, nbyte);
    if (written < 0 ) {
        rt_printf("RTSerial::write(): error on rt_dev_write(), %s\n", strerror(-written));
    } else if (static_cast<size_t>(written) != nbyte) {
        rt_printf("RTSerial::write(): only %d / %d byte transmitted\n", written, nbyte);
    }
    return written;
}

int RTSerial::flush() {

    long purge_mask = RTDM_PURGE_RX_BUFFER | RTDM_PURGE_TX_BUFFER;
    int err = rt_dev_ioctl(fd_ser, RTIOC_PURGE,  purge_mask);
    if (err) {
        rt_printf("error on RTIOC_PURGE, %s\n", strerror(-err));
        return err; 
    }

}
    
