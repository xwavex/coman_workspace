/*
IMPORTANT: because of the use of inline functions, you *have* to use
'-O' or some variation when you compile your program!
*/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <time.h>
#include <signal.h>
#include <errno.h>
#include <syslog.h>

#include "linux/i2c-dev.h"

#define EXIT_SUCCESS 0
#define EXIT_FAILURE 1

#define DEFAULT_ADAPTER_NUMBER  8
#define BATTERY_I2C_ADDR        0x0B
#define POWER_BTN_I2C_ADDR      0X1B
#define POWER_BTN_OFF           0x59

typedef float (*scalefun_t)(int);

typedef struct {
    __u8    reg;
    char    valid;
    int     value;
    const char * reg_name;
    const char * unit;
    scalefun_t scale_fun;
} battery_reg_t;


static void daemonize(void)
{
    FILE * dummy;
    pid_t pid, sid;

    /* already a daemon */
    if ( getppid() == 1 ) return;

    /* Fork off the parent process */
    pid = fork();
    if (pid < 0) {
        exit(EXIT_FAILURE);
    }
    /* If we got a good PID, then we can exit the parent process. */
    if (pid > 0) {
        exit(EXIT_SUCCESS);
    }

    /* At this point we are executing as the child process */

    signal(SIGCHLD,SIG_DFL); /* A child process dies */
    signal(SIGHUP, SIG_IGN); /* Ignore hangup signal */
    signal(SIGTERM,SIG_DFL); /* Die on SIGTERM */

    /* Change the file mode mask */
    umask(0);

    /* Create a new SID for the child process */
    sid = setsid();
    if (sid < 0) {
        exit(EXIT_FAILURE);
    }

    /* Change the current working directory.  This prevents the current
       directory from being locked; hence not being able to remove it. */
    if ((chdir("/")) < 0) {
        exit(EXIT_FAILURE);
    }

    /* Redirect standard files to /dev/null */
    dummy = freopen( "/dev/null", "r", stdin);
    dummy = freopen( "/dev/null", "w", stdout);
    dummy = freopen( "/dev/null", "w", stderr);
}

static int shutdown() {

    /*
    system("echo 1 > /proc/sys/kernel/sysrq");
    //Send a SIGTERM to all processes, except for init
    system("echo t > /proc/sysrq-trigger");
    //Send a SIGKILL to all processes, except for init
    system("echo i > /proc/sysrq-trigger");
    //  Will attempt to sync all mounted filesystems
    system("echo s > /proc/sysrq-trigger");
    // Will attempt to remount all mounted filesystems read-only
    system("echo u > /proc/sysrq-trigger");
    */
    int ret = system("/sbin/poweroff");
    return ret;
}


static int poll_battery(int file, int addr, battery_reg_t *batt_regs, int size) {

    int i;
    int res;

    res = ioctl(file,I2C_SLAVE,addr);
    if (res < 0) {
        /* ERROR HANDLING; you can check errno to see what went wrong */
        syslog( LOG_ERR,  "ioctl failed on address %d : %d\n", addr, res);
        return res;
    }

    /* Using SMBus commands */
    for (i=0; i < size; i++) {
        res = i2c_smbus_read_word_data(file, batt_regs[i].reg);
        if (res < 0) {
            /* ERROR HANDLING: i2c transaction failed */
            batt_regs[i].valid = res; 
            syslog( LOG_ERR, "I2C transaction failed at reg %d\n", batt_regs[i].reg);
        } else {
            /* res contains the read word */
            batt_regs[i].valid = 1;
            batt_regs[i].value = res;
            //syslog( LOG_INFO,"Reg 0x%02X: Read 0x%04X %5d\n", batt_regs[i].reg, res, res);
        }
        // breath
        usleep(100000);
    }

    return 0;
}


static int poll_power_btn(int file, int addr, unsigned char reg, unsigned char *reg_value) {

    int res;

    res = ioctl(file,I2C_SLAVE,addr);
    if (res < 0) {
        /* ERROR HANDLING; you can check errno to see what went wrong */
        syslog( LOG_ERR, "ioctl failed on address %d : %d\n", addr, res);
        return res;
    }

    res = i2c_smbus_read_byte_data(file, reg);
    *reg_value = res;
    if (res < 0) {
        syslog( LOG_ERR, "I2C transaction failed at reg %d\n", reg);
    }

    return 0;
}

float toCelsius(int K) { return (float)K/10-273.15; }

float toVolt(int V) { return (float)V/1000; }

float toAmp(int A) { return (float)((signed short)A)/100; }

int main(int argc, char **argv)
{
    int file;
    int nbytes;
    char filename[20], buffer[1024], * pbuffer;
    const char * pipes_name[] = { "/tmp/battery", "/tmp/battery_raw"};
    int adapter_nr = DEFAULT_ADAPTER_NUMBER;
    int i, fd_batt, fd_batt_raw;
    unsigned char power_btn;
    time_t t;
    struct tm tm;
    
    const char * battery_reg_name[] = {
        "Temperature",
        "Voltage",
        "Current",
        "RelativeStatusOfCharge",
        "AbsoluteStatusOfCharge",
        "AverageTimeToEmpty",
        "ChargingVoltage",
        "ChargingCurrent",
        "AverageTimeToFull",
        "BatteryStatus",
        "Cell Volt 1",
        "Cell Volt 2",
        "Cell Volt 3",
        "Cell Volt 4",
        "Cell Volt 5",
        "Cell Volt 6",
        "Cell Volt 7"
    };

    // see http://www.ti.com/lit/ug/sluu481/sluu481.pdf for registry description

    i=0;
    battery_reg_t battery_regs[] = {
        { 0x08, 0, 0, battery_reg_name[0], "C", toCelsius},
        { 0x09, 0, 0, battery_reg_name[1], "V", toVolt},
        { 0x0A, 0, 0, battery_reg_name[2], "A", toAmp},
        { 0x0D, 0, 0, battery_reg_name[3], "%", 0},
        { 0x0E, 0, 0, battery_reg_name[4], "%", 0},
        { 0x12, 0, 0, battery_reg_name[5], "min", 0},
        { 0x15, 0, 0, battery_reg_name[6], "V", toVolt},
        { 0x14, 0, 0, battery_reg_name[7], "A", toAmp},
        { 0x13, 0, 0, battery_reg_name[8], "min", 0},
        { 0x16, 0, 0, battery_reg_name[9], "bits", 0},
        { 0x3C, 0, 0, battery_reg_name[10], "V", toVolt},
        { 0x3D, 0, 0, battery_reg_name[11], "V", toVolt},
        { 0x3E, 0, 0, battery_reg_name[12], "V", toVolt},
        { 0x3F, 0, 0, battery_reg_name[13], "V", toVolt},
        { 0x40, 0, 0, battery_reg_name[14], "V", toVolt},
        { 0x41, 0, 0, battery_reg_name[15], "V", toVolt},
        { 0x42, 0, 0, battery_reg_name[16], "V", toVolt}
    };

    while ((i=getopt(argc,argv,"b:"))>=0) {
        switch (i) {
            case 'b':
                adapter_nr = atoi(optarg);
                break;
            default:
                break;
        }
    }


    if ( adapter_nr < 0 || adapter_nr > 0xff ) {

        exit(EXIT_FAILURE);
    }

    daemonize();

    openlog("ccub-battery-daemon", LOG_PID, LOG_LOCAL5 );
    syslog( LOG_INFO, "using i2c bus %d\n", adapter_nr );


    sprintf(filename,"/dev/i2c-%d",adapter_nr);
    if ((file = open(filename,O_RDWR)) < 0) {
        syslog( LOG_ERR, "couldn't open file %s\n", filename);
        exit(EXIT_FAILURE);
    }

    mkfifo(pipes_name[0], S_IROTH);
    fd_batt = open(pipes_name[0], O_RDWR | O_NONBLOCK);
    mkfifo(pipes_name[1], S_IROTH);
    fd_batt_raw = open(pipes_name[1], O_RDWR | O_NONBLOCK);

    for (;;) {

        sleep(1);

        // poll battery
        poll_battery(file, BATTERY_I2C_ADDR, battery_regs, sizeof(battery_regs)/sizeof(battery_reg_t));

        // write to pipes binary battery_regs 
        nbytes = write(fd_batt_raw, (void*)&battery_regs, sizeof(battery_regs));
        pbuffer = buffer;
        // produce a human readable output
        t = time(NULL);
        tm = *localtime(&t);
        pbuffer += sprintf(pbuffer,"%d-%d-%d %d:%d:%d\n", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);

        for (i=0; i < sizeof(battery_regs)/sizeof(battery_reg_t); i++) {
            // check read error
            if (battery_regs[i].valid <= 0) {
                pbuffer += sprintf(pbuffer, "%s: Read error\n", battery_regs[i].reg_name);
            } else {
                pbuffer += sprintf(pbuffer, "%s: ", battery_regs[i].reg_name);
                if (battery_regs[i].scale_fun) {
                    pbuffer += sprintf(pbuffer, "%.3f ", battery_regs[i].scale_fun(battery_regs[i].value));
                } else {
                    pbuffer += sprintf(pbuffer, "%d ", battery_regs[i].value);
                }
                pbuffer += sprintf(pbuffer, "%s\n", battery_regs[i].unit);
            }
        }
        pbuffer += sprintf(pbuffer, "\n");

        // write to pipes human readable output
        nbytes = write(fd_batt, (void*)buffer, pbuffer-buffer);

        // poll power button status reading 0x1A register
        poll_power_btn(file,POWER_BTN_I2C_ADDR, 0x1A, &power_btn);

        // test poweroff condition 
        if ( power_btn == POWER_BTN_OFF ) {

            shutdown();
            break;
        }
    }

    // clean up ....
    close(fd_batt);
    close(fd_batt_raw);

    // tell the battery to switch off reading 0xBD registser
    // 5 sec ... not enougth
    poll_power_btn(file,POWER_BTN_I2C_ADDR, 0xBD, &power_btn);

    syslog( LOG_NOTICE, "Done ..." );
    closelog();

    exit(EXIT_SUCCESS);
}

