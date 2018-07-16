#include <signal.h>
#include <sys/mman.h>
#include <bits/local_lim.h>

#include <definitions.h>
#include <thread_util.h>
#include <utils.h>

#include <iostream>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/count.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/min.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/variance.hpp>
#include <boost/accumulators/statistics/error_of.hpp>
#include <boost/accumulators/statistics/error_of_mean.hpp>

#include <imu_3DM-GX3-25.h>

static int loop = 1;

using namespace boost::accumulators;



class IMU_thread : public Thread_hook {

#if __XENO__
    rt_imu_3DM_GX3_25  * imu;
#else
    nrt_imu_3DM_GX3_25  * imu;
#endif

    float               acc[3], angRate[3], mag[3], orientMat[9], quat[4], euler[3];
    data_3DM_GX3_t      raw_data;
    unsigned long long  tNow, dt;

    accumulator_set<uint64_t, 
    features<
        tag::count
        ,tag::mean
        ,tag::min
        ,tag::max
        ,tag::variance(lazy)
        ,tag::error_of<tag::mean>
        >
    > loop_time;


public:

    IMU_thread() {

        name = "IMU_thread";
        period.period = {0,1}; 
        
#if __XENO__
        schedpolicy = SCHED_FIFO;
#else
        schedpolicy = SCHED_OTHER;
#endif
        priority = sched_get_priority_max(schedpolicy)/2;
        stacksize = PTHREAD_STACK_MIN;
    }

    ~IMU_thread() {

        delete imu;
        std::cout << "Count loop time : " << (uint64_t)count(loop_time) << std::endl;
        std::cout << "Mean loop time  ns : " << (uint64_t)mean(loop_time) << std::endl;
        std::cout << "Min loop time ns : " << min(loop_time) << std::endl;
        std::cout << "Max loop time ns : " << max(loop_time) << std::endl;
        std::cout << "Var loop time ns : " << variance(loop_time) << std::endl;
        std::cout << "Mean error loop time ns : " << boost::accumulators::error_of<tag::mean>(loop_time) << std::endl;

    }

    virtual void th_init(void *) { 

        int imu_cmd = 
            //0;
            CMD_EULER;
            //CMD_ACCEL_ANGRATE;
            //CMD_QUATERNION;
            //CMD_ACCEL_ANGRATE_MAG_ORIENT;

#if __XENO__
        imu =  new rt_imu_3DM_GX3_25("rtser0", 115200, imu_cmd);
#else
        imu = new nrt_imu_3DM_GX3_25("/dev/ttyACM0", 115200, imu_cmd);
#endif

    }

    virtual void th_loop(void *) { 

        tNow = get_time_ns();
        // busy wait on condition variable !!! if imu thread rx a valid msg
        imu->get_raw_data_sync(raw_data);

        imu->get_Euler(euler, 0);
        //imu->get_Quaternion(quat, 0);
        //imu->get_Acc_Ang(acc, angRate, 0);
        //imu->get_Acc_Ang_Orient(acc, angRate, orientMat, 0);
        //imu->get_Acc_Ang_Mag_Orient(acc, angRate, mag, orientMat, 0);

        dt = get_time_ns()-tNow;
        loop_time(dt);

#if 1
        //DPRINTF("loop time %lld\n", dt);
        //DPRINTF("quat %.6f %.6f %.6f %.6f\n", quat[0], quat[1], quat[2], quat[3]);
        DPRINTF("euler %.6f %.6f %.6f\n", euler[0], euler[1], euler[2]);
        //DPRINTF("acc %.6f %.6f %.6f\n", acc[0], acc[1], acc[2]);
        //DPRINTF("ang %.6f %.6f %.6f\n", angRate[0], angRate[1], angRate[2]);
        //DPRINTF("mag %.6f %.6f %.6f\n", mag[0], mag[1], mag[2]);
#if 0
        DPRINTF("oM [%.6f %.6f %.6f\n", orientMat[0], orientMat[1], orientMat[2]);
        DPRINTF("   [%.6f %.6f %.6f\n", orientMat[3], orientMat[4], orientMat[5]);
        DPRINTF("   [%.6f %.6f %.6f]\n", orientMat[6], orientMat[7], orientMat[8]);
#endif
#endif

    }

};




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

    // set signal handler
    set_signal_handler();

    IMU_thread imu_thread;

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
   

    /////////////////////////////////////////////////////////////////
    //
    /////////////////////////////////////////////////////////////////
    imu_thread.create();

    while (loop) {
        sleep(1);
    }

    // stop thread loop
    imu_thread.stop();
    // wait test_thread termination
    imu_thread.join();

    DPRINTF("Exit main\n");

    return 0;

}



