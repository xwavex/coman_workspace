#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <errno.h>
#include <assert.h>
#include <execinfo.h>

#include <pthread.h>
#include <bits/local_lim.h>

#ifdef RT_ENV
    #include <rtdk.h>
    #define DPRINTF rt_printf
#else
    #define DPRINTF printf
#endif


int loop = 1;


////////////////////////////////////////////////////
// Prototype functions
////////////////////////////////////////////////////

unsigned long long get_time_ns(void);

extern void * test_boards_thread(void *);
#ifdef USE_ZMQ
    extern void * zmq_pub_thread(void*);
#endif
////////////////////////////////////////////////////
// Static functions
////////////////////////////////////////////////////

static void stop_loop(int sig)
{
    loop = 0;
    printf("going down ... loop %d\n", loop);
}

static void warn_upon_switch(int sig __attribute__((unused)))
{
    // handle rt to nrt contex switch
    void *bt[32];
    int nentries;

    /* Dump a backtrace of the frame which caused the switch to
       secondary mode: */
    nentries = backtrace(bt,sizeof(bt)/sizeof(bt[0]));
    // dump backtrace 
    //backtrace_symbols_fd(bt,nentries,fileno(stdout));
}

static void set_signal_handler(void)
{
    signal(SIGINT,  stop_loop);
    //signal(SIGKILL, stop_loop);
    signal(SIGTERM, stop_loop);
#ifdef RT_ENV
    // call pthread_set_mode_np(0, PTHREAD_WARNSW) to cause a SIGXCPU
    // signal to be sent when the calling thread involontary switches to secondary mode
    signal(SIGXCPU, warn_upon_switch);
#endif
}


////////////////////////////////////////////////////
// Main
////////////////////////////////////////////////////


int main(int argc, char *argv[])
{
    // set signal handler
    set_signal_handler();

#ifdef RT_ENV
    /* Prevent any memory-swapping for this program */
    int ret = mlockall(MCL_CURRENT | MCL_FUTURE);
    if (ret < 0) {
        printf("mlockall failed (ret=%d) %s\n", ret, strerror(ret));
        exit(1);
    }

    /*
     * This is a real-time compatible printf() package from
     * Xenomai's RT Development Kit (RTDK), that does NOT cause
     * any transition to secondary (i.e. non real-time) mode when
     * writing output.
     */
    rt_print_auto_init(1);
#endif
    
    /////////////////////////////////////////////////////////////////
    //
    /////////////////////////////////////////////////////////////////
    pthread_t           th1, th2;
    pthread_attr_t      attr;
    int                 policy;
    cpu_set_t           cpu_set;
    struct sched_param  schedparam;

    CPU_SET(1,&cpu_set);

#ifdef RT_ENV
    policy = SCHED_FIFO;
#else
    policy = SCHED_FIFO;
    //policy = SCHED_OTHER;
#endif

    // thread configuration and creation
    pthread_attr_init(&attr);
    pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(&attr, policy);
    schedparam.sched_priority = sched_get_priority_max(policy);
    pthread_attr_setschedparam(&attr, &schedparam);
    pthread_attr_setstacksize(&attr, PTHREAD_STACK_MIN);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
    //pthread_attr_setaffinity_np(&attr, sizeof(cpu_set), &cpu_set);
    pthread_create(&th1, &attr, test_boards_thread, (void*)0);
    pthread_attr_destroy(&attr);

#ifdef USE_ZMQ
    // thread configuration and creation
    pthread_attr_init(&attr);
    pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(&attr, SCHED_OTHER);
    schedparam.sched_priority = sched_get_priority_max(SCHED_OTHER);
    pthread_attr_setschedparam(&attr, &schedparam);
    pthread_attr_setstacksize(&attr, PTHREAD_STACK_MIN);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
    //pthread_attr_setaffinity_np(&attr, sizeof(cpu_set), &cpu_set);
    pthread_create(&th2, &attr, zmq_pub_thread, (void*)0);
    pthread_attr_destroy(&attr);
#endif

    while (loop) {
        sleep(1);
    }

#ifdef USE_ZMQ
    // wait test_thread termination
    pthread_cancel(th2);
    pthread_join(th2, 0);
#endif
    // wait test_thread termination
    pthread_join(th1, 0);
    DPRINTF("Exit main\n");

    return 0;
}

