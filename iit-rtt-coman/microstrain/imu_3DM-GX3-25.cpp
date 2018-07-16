#include <assert.h>
#include <string.h>

#ifndef __APPLE__
#include <bits/local_lim.h>
#include <linux/sched.h>
#else
#include <limits.h>
#endif

#include "imu_3DM-GX3-25.h"

// NOTE : speed is fix to 115200/921600. this is the default and max speed with rs232
// setting continuous mode is faster but dependend also on which command we set
// for example setting a command of length 31 bytes gives an output rate of 250 Hz ( see page 40 of manual )
// with the longest cmd 0xCC we get 100 Hz ....
// with quaternion  cmd 0xDF we get 333 Hz ....

pthread_mutex_t imu_3DM_GX3_25::data_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t imu_3DM_GX3_25::sync_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t  imu_3DM_GX3_25::sync_cond = PTHREAD_COND_INITIALIZER;

///////////////////////////////////////////////////////////////////////////////
//
///////////////////////////////////////////////////////////////////////////////
imu_3DM_GX3_25::imu_3DM_GX3_25(void) {

    _continuous = false;
    C2_cmd.cmd = CMD_ACCEL_ANGRATE;
    C2_cmd.expSize = 31;
    C2_cmd.process = process_C2;


    C8_cmd.cmd = CMD_ACCEL_ANGRATE_ORIENT;
    C8_cmd.expSize = 67;
    C8_cmd.process = process_C8;

    CC_cmd.cmd = CMD_ACCEL_ANGRATE_MAG_ORIENT;
    CC_cmd.expSize = 79;
    CC_cmd.process = process_CC;

    CE_cmd.cmd = CMD_EULER;
    CE_cmd.expSize = 19;
    CE_cmd.process = process_CE;

    CF_cmd.cmd = CMD_EULER_ANGRATE;
    CF_cmd.expSize = 31;
    CF_cmd.process = process_CF;

    DF_cmd.cmd = CMD_QUATERNION;
    DF_cmd.expSize = 23;
    DF_cmd.process = process_DF;

    // if no continuous_mode is set ... imu worker thread cycle over these ....
    cmd_ptr_map[CMD_ACCEL_ANGRATE]              = &C2_cmd;
    cmd_ptr_map[CMD_ACCEL_ANGRATE_ORIENT]       = &C8_cmd;
    cmd_ptr_map[CMD_ACCEL_ANGRATE_MAG_ORIENT]   = &CC_cmd;
    cmd_ptr_map[CMD_EULER]                      = &CE_cmd;
    cmd_ptr_map[CMD_EULER_ANGRATE]              = &CF_cmd;
    cmd_ptr_map[CMD_QUATERNION]                 = &DF_cmd;

}

void imu_3DM_GX3_25::get_Acc_Ang(float acc[3], float angRate[3], uint64_t *time) {

    acc_angRate_t * aa;

    pthread_mutex_lock(&data_mutex);
    aa = &C2_cmd.data.aa;
    if (acc) {
        memcpy((void*)acc, aa->acc._bytes, sizeof(float)*3);
    }
    if (angRate) {
        memcpy((void*)angRate, aa->angRate._bytes, sizeof(float)*3);
    }
    pthread_mutex_unlock(&data_mutex);
}

void imu_3DM_GX3_25::get_Acc_Ang_Orient(float acc[3], float angRate[3], float orientMat[9], uint64_t *time) {

    acc_ang_orient_t * aaom;

    pthread_mutex_lock(&data_mutex);
    aaom = &C8_cmd.data.aaom;
    if (acc) {
        memcpy((void*)acc, aaom->acc._bytes, sizeof(float)*3);
    }
    if (angRate) {
        memcpy((void*)angRate, aaom->angRate._bytes, sizeof(float)*3);
    }
    if (orientMat) {
        memcpy((void*)orientMat, aaom->orientMat._bytes, sizeof(float)*9);
    }
    pthread_mutex_unlock(&data_mutex);
}

void imu_3DM_GX3_25::get_Acc_Ang_Mag_Orient(float acc[3], float angRate[3], float mag[3], float orientMat[9], uint64_t *time) {

    acc_ang_mag_orient_t * aamom;

    pthread_mutex_lock(&data_mutex);
    aamom = &CC_cmd.data.aamom;
    if (acc) {
        memcpy((void*)acc, aamom->acc._bytes, sizeof(float)*3);
    }
    if (angRate) {
        memcpy((void*)angRate, aamom->angRate._bytes, sizeof(float)*3);
    }
    if (mag) {
        memcpy((void*)mag, aamom->mag._bytes, sizeof(float)*3);
    }
    if (orientMat) {
        memcpy((void*)orientMat, aamom->orientMat._bytes, sizeof(float)*9);
    }
    pthread_mutex_unlock(&data_mutex);
}

void imu_3DM_GX3_25::get_Euler(float euler[3], uint64_t *time) {

    eul_t * eu;

    pthread_mutex_lock(&data_mutex);
    eu = &CE_cmd.data.eu;
    if (euler) {
        memcpy((void*)euler, eu->eul._bytes, sizeof(float)*3);
    }
    pthread_mutex_unlock(&data_mutex);
}

void imu_3DM_GX3_25::get_Euler_AngularRate(float euler[3], float angRate[3], uint64_t *time) {

    eul_angRate_t * ea;

    pthread_mutex_lock(&data_mutex);
    ea = &CE_cmd.data.ea;
    if (euler) {
        memcpy((void*)euler, ea->eul._bytes, sizeof(float)*3);
    }
    if (angRate) {
        memcpy((void*)angRate, ea->angRate._bytes, sizeof(float)*3);
    }
    pthread_mutex_unlock(&data_mutex);
}

void imu_3DM_GX3_25::get_Quaternion(float quat[4], uint64_t *time) {

    quat_t * q;

    pthread_mutex_lock(&data_mutex);
    q = &DF_cmd.data.quat;
    if (quat) {
        memcpy((void*)quat, q->quat._bytes, sizeof(float)*4);
    }
    pthread_mutex_unlock(&data_mutex);
}

void imu_3DM_GX3_25::get_raw_data_sync(data_3DM_GX3_t &) {

    pthread_mutex_lock(&sync_mutex);
    pthread_cond_wait(&sync_cond, &sync_mutex);
    pthread_mutex_unlock(&sync_mutex);

}

void imu_3DM_GX3_25::printHex(uint8_t *buff, int len) {

    for(auto i=0; i<len; i++) {
        DPRINTF("0x%02X ",buff[i]);
    }
    DPRINTF("\n");

}


void * imu_3DM_GX3_25::imu_worker(void *_) {

    static struct timespec ts = {0, 1000000};  // 1ms 1kHz
//    static struct timespec ts = {0, 2000000}; // 2ms 500Hz
    //static struct timespec ts = {0, 4000000};  // 4ms 250Hz
    //static struct timespec ts = {0, 5000000};  // 5ms 200Hz
//    static struct timespec ts = {0, 10000000}; // 10ms 100Hz
    static data_3DM_GX3_t   th_data;
    int nbytes;

    imu_3DM_GX3_25 * kls = (imu_3DM_GX3_25*)_;
    imu_cmd_t * tmp_cmd = NULL;
    auto it = kls->cmd_ptr_map.begin();

    DPRINTF("Start imu worker ....\n");
#ifdef __XENO__
    pthread_set_mode_np(0, PTHREAD_WARNSW);
    pthread_set_name_np(pthread_self(), "imu_worker");
#endif

    for (;;) {
#ifdef __APPLE__
        nanosleep(&ts, NULL);
#else
        clock_nanosleep(CLOCK_REALTIME, 0, &ts, NULL);
#endif

        if ( kls->_continuous ) {

            tmp_cmd = kls->_continuous_cmd_ptr;
            // read continuos command
            nbytes = kls->read((void*)&th_data.buffer, tmp_cmd->expSize);
        } else {
            tmp_cmd = it->second;
            // transmit command
            nbytes = kls->write((void*)&tmp_cmd->cmd, sizeof(uint8_t));
            // receive response
            nbytes = kls->read((void*)&th_data.buffer, tmp_cmd->expSize);
        }

        if ( nbytes < 0 ) {
            // continue loop ...
            continue;

        } else if ( nbytes != tmp_cmd->expSize ) {
            DPRINTF("read %d instead of %d\n", nbytes, tmp_cmd->expSize);
            // continue loop ...
            continue;

        } else {

            //kls->printHex(th_data.buffer,  nbytes);

            // process data ... at least swap bytes !!!
            if ( tmp_cmd->process ) {
                // compute checksum and swap bytes
                if ( ! (bool)tmp_cmd->process(th_data) ) {
                    DPRINTF("Checksum FAILURE\n");
                    continue;
                }
            }

            // copy for consumer ....
            pthread_mutex_lock(&data_mutex);
            memcpy((void*)&tmp_cmd->data.buffer, &th_data.buffer, tmp_cmd->expSize);
            pthread_mutex_unlock(&data_mutex);

            pthread_mutex_lock(&sync_mutex);
            pthread_cond_signal(&sync_cond);
            pthread_mutex_unlock(&sync_mutex);

        }

        // loop over commands
        it++;
        if (it == kls->cmd_ptr_map.end()) {
            it = kls->cmd_ptr_map.begin();
        }

    }

    return 0;
}

void imu_3DM_GX3_25::start_continuous(uint8_t cmd_byte) {

    int nbytes;
    uint8_t buff[] = {
        CMD_CONTINUOUS,
        0xC1, 0x29, // confirms user intent
        0
    };

    //  lookup cmds
    _continuous_cmd_ptr = (cmd_ptr_map.find(cmd_byte) != cmd_ptr_map.end()) ? cmd_ptr_map[cmd_byte] : NULL;
    if ( ! _continuous_cmd_ptr) {
        DPRINTF("Imu polling defined commands\n");
        return;
    }

    _continuous = true;
    buff[3] = _continuous_cmd_ptr->cmd;

    nbytes = write((void*)buff, sizeof(buff));
    nbytes = read((void*)buff, 8);

    DPRINTF("%d  0x%02X  0x%02X\n", nbytes, buff[1] , _continuous_cmd_ptr->cmd);
    if (buff[1] != _continuous_cmd_ptr->cmd ) {
        sleep(1);
        assert(0);
    }

    DPRINTF("Imu set continuous mode 0x%02X\n", _continuous_cmd_ptr->cmd);
}

void imu_3DM_GX3_25::stop_continuous(void) {

    int nbytes;
    const uint8_t buff[] = {
        CMD_STOP_CONTINUOUS,
        0x75, 0xB4 // confirms user intent
    };

    nbytes = write((void*)buff, sizeof(buff));

    if (nbytes != sizeof(buff))
    {
        DPRINTF("Failed to disable continuous mode\n");
    }
    // !!! need to clean serial buffer ...
    sleep(1);
    flush();
    _continuous = false;
    DPRINTF("Imu stop continuous\n");

}

void imu_3DM_GX3_25::sample_setting(void) {

    int nbytes;

    // my code to change baud rate
    uint8_t buff_test[11] = {
        0xD9, 0xC3, 0x55,
        0x01, 0x00, // change the last one to 0x02 to write to EEPROM
        0x00, 0x0E, 0x10, 0x00,
        0x02,
        0x00
    };
    uint8_t reply_test[10];

    nbytes = write((void*)buff_test, 11);
    if (nbytes != 11)
        DPRINTF("Failed to set test\n");
    nbytes = read((void*)reply_test, 10);

    nbytes = write((void*)buff_test, 11);
    if (nbytes != 11)
        DPRINTF("Failed to set test\n");
    nbytes = read((void*)reply_test, 10);
    if(nbytes==10 &&
       buff_test[5]==reply_test[2] &&
       buff_test[6]==reply_test[3] &&
       buff_test[7]==reply_test[4] &&
       buff_test[8]==reply_test[5])
       DPRINTF("Baudrate successfully changed to 921600\n");
//    printHex(reply_test, nbytes);
    // end of my code


    uint8_t reply[20];
    uint8_t buff[20] = {
        CMD_SAMPLING_SETTING,
        0xA8, 0xB9, // confirm user intent
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
    };

    nbytes = write((void*)buff, 20);

    if (nbytes != 20)
    {
        DPRINTF("Failed to set sample setting\n");
    }

    nbytes = read((void*)reply, 19);

    if (nbytes != 19)
    {
        DPRINTF("Failed to read sample setting reply\n");
    }
//    printHex(reply, 19);

    //assert(buff[0] == reply[0]);
    //printHex(reply, 19);

    memcpy(buff+4, reply+1, 10);

    // Function selector : change parameter but do not send a reply
    buff[3]  = 3;



    // my code to change data rate division number (refer to manual)
    // http://files.microstrain.com/3DM-GX3-25%20Single%20Byte%20Data%20Communications%20Protocol.pdf
    // check 0xDB command set
    buff[5]  = 0x02; // 500 is the maximum recommended data rate when rotation calculation is also demanded.
    // end of my code



    // Data conditioning function selector byte 7-8 : bit 12 enable quaternion ... remember big endian
    buff[6] |= 16;

    // disable magnetometer
    buff[6] |= 1;
    buff[6] |= 4;

//    printHex(buff, 20);


    nbytes = write((void*)buff, 20);

    usleep(100000);

}

void imu_3DM_GX3_25::start_worker() {

    pthread_attr_t      attr;
    //cpu_set_t           cpu_set;
    struct sched_param  schedparam;

    pthread_mutex_init(&data_mutex, NULL);
    pthread_mutex_init(&sync_mutex, NULL);
    pthread_cond_init(&sync_cond, NULL);

#if __XENO__
    int schedpolicy = SCHED_FIFO;
#else
    int schedpolicy = SCHED_OTHER;
#endif

    // thread configuration and creation
    pthread_attr_init(&attr);
    pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(&attr, schedpolicy);
    schedparam.sched_priority = sched_get_priority_max(schedpolicy)/2;
    pthread_attr_setschedparam(&attr, &schedparam);
    pthread_attr_setstacksize(&attr, PTHREAD_STACK_MIN);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
    //CPU_SET(2,&cpu_set);
    //pthread_attr_setaffinity_np(&attr, sizeof(cpu_set), &cpu_set);
    pthread_create(&worker_thread, &attr, imu_worker, (void*)this);
    pthread_attr_destroy(&attr);

}

void imu_3DM_GX3_25::stop_worker() {

    pthread_cancel(worker_thread);
    pthread_join(worker_thread, 0);
    pthread_mutex_destroy(&data_mutex);
    pthread_mutex_destroy(&sync_mutex);
    pthread_cond_destroy(&sync_cond);
    DPRINTF("Exit imu worker ....\n");
}

///////////////////////////////////////////////////////////////////////////////
//
///////////////////////////////////////////////////////////////////////////////

nrt_imu_3DM_GX3_25::nrt_imu_3DM_GX3_25(const std::string& path, int baudrate, uint8_t continuous_cmd):
NRTSerial(path.c_str(), baudrate) {

    if (open() < 0)
    {
        return;
    }

    stop_continuous();
    sample_setting();
    if ( continuous_cmd ) {
        start_continuous(continuous_cmd);
    }
    // note that start_continuous and stop_continuous use the serial fd and also the thread use it !!!!
    // no mutex are required until they are called before and after threads life

    // now start thread
    start_worker();

}

nrt_imu_3DM_GX3_25::~nrt_imu_3DM_GX3_25() {
    // if (fd() >= 0)
    // {
    //     stop_worker();
    //     stop_continuous();
    //     close();
    // }
}

///////////////////////////////////////////////////////////////////////////////
//
///////////////////////////////////////////////////////////////////////////////
#ifdef __XENO__

    rt_imu_3DM_GX3_25::rt_imu_3DM_GX3_25(const std::string& path, int baudrate, uint8_t continuous_cmd):
    RTSerial(path.c_str(), baudrate, 1e9, RTDM_TIMEOUT_INFINITE) {

        if (open() < 0)
        {
            return;
        }

        stop_continuous();
        sample_setting();
        if ( continuous_cmd ) {
            start_continuous(continuous_cmd);
        }
        // note that start_continuous and stop_continuous use the serial fd and also the thread use it !!!!
        // no mutex are required until they are called before and after threads life

        // now start thread
        start_worker();

    }

    rt_imu_3DM_GX3_25::~rt_imu_3DM_GX3_25() {
        if (fd() >= 0)
        {
            stop_worker();
            stop_continuous();
            close();
        }
    }

    int rt_imu_3DM_GX3_25::read(void *buf, size_t nbyte) {

        int nb = RTSerial::read(buf,nbyte,&rx_event);
        //rx_evt_info(buf,nbyte);
        return nb;
    }

    int rt_imu_3DM_GX3_25::write(const void *buf, size_t nbytes) {

        return RTSerial::write(buf,nbytes);
    }

    void rt_imu_3DM_GX3_25::rx_evt_info(void *buf, size_t nbyte) {

        /*
        int events : signalled events, see RTSER_EVENT_xxx
        int rx_pending : number of pending input characters
        nanosecs_abs_t 	last_timestamp : last interrupt timestamp
        nanosecs_abs_t 	rxpend_timestamp : reception timestamp of oldest character in input queue
        */

        DPRINTF("\trx_event.events %d\n", rx_event.events);
        DPRINTF("\trx_event.rx_pending %d\n", rx_event.rx_pending);
        // last interrupt timestamp
        DPRINTF("\trx_event.last_timestamp   %lld\n", rx_event.last_timestamp);
        // reception timestamp of oldest character in input queue
        DPRINTF("\trx_event.rxpend_timestamp %lld\n", rx_event.rxpend_timestamp);
        DPRINTF("\tdelta %lld\n", rx_event.last_timestamp-rx_event.rxpend_timestamp);
        DPRINTF("\texpected size %d\n", nbyte);
    }

#endif
