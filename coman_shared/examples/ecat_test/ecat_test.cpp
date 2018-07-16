#include <signal.h>
#include <sys/mman.h>
#include <bits/local_lim.h>
#include <execinfo.h>

#include <definitions.h>
#include <thread_util.h>
#include <utils.h>

#include <beckhoff/ECM_beckhoff.h>
#include <shadow/ECM_shadow.h>
#include <advr_iit/ECM_test.h>

static int loop = 1;


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
#ifdef __XENO__
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
    int ret;
    std::map<std::string, Thread_hook*> threads;

    // set signal handler
    set_signal_handler();

#ifdef BECKHOFF
    threads["ecat_master"] = new ECat_master_beckhoff();
#elif SHADOW
    threads["ecat_master"] = new ECat_master_shadow();
#elif ADVR
    threads["ecat_master"] = new ECat_master_test();
#else
    #error "Define at least one ECat_master implementation"
#endif

#ifdef __XENO__
    /* Prevent any memory-swapping for this program */
    ret = mlockall(MCL_CURRENT | MCL_FUTURE);
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

    threads["ecat_master"]->create();

    while (loop) {
        sleep(1);
    }

    for (auto it = threads.begin(); it != threads.end(); it++) {
        it->second->stop();
        it->second->join();
        delete it->second;
    }

    DPRINTF("Exit main\n");

    return 0;

}


