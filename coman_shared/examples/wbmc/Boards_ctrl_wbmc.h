/*
   Boards_ctrl_basic.h

   Copyright (C) 2012 Italian Institute of Technology

   Developer:
       Alessio Margan (2013-, alessio.margan@iit.it)

*/

/**
 *
 * @author Alessio Margan (2013-, alessio.margan@iit.it)
*/

#ifndef __BOARDS_CTRL_BASIC_H__
#define __BOARDS_CTRL_BASIC_H__

#include <utils.h>
#include <thread_util.h>

#include <Boards_ctrl_ext.h>
#include <imu_3DM-GX3-25.h>

#include <Eigen/Dense>
using namespace Eigen;

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/count.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/min.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/variance.hpp>
#include <boost/accumulators/statistics/error_of.hpp>
#include <boost/accumulators/statistics/error_of_mean.hpp>

typedef struct {
    uint64_t    ts;
    float       q[29];
    float       sinq[29];
    float       cosq[29];
    float       tau_GC[29];
    float       epsq;
    float       effort;
    float       k;
} min_effort_args_t;


typedef struct {
    uint64_t    ts;
    uint64_t    dt;
    int         i;
    float       min_effort;
} min_effort_res_t;

/**
 * @class Boards_ctrl_wbmc
 * @defgroup Boards_controller Boards_controller
 * @ingroup RoboLLI
 * @brief Boards_ctrl_basic class
 */

class Boards_ctrl_wbmc : public Thread_hook, public Boards_ctrl_ext  {

private:

    //std::string pipes_name[2] = { "parallel_calc_res", "parallel_calc_arg" };
    std::string pipes_name[2];
    int         fd_calc_read_res, fd_calc_write_arg;

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

    uint64_t  tNow, dt, g_tStart;

    boost::circular_buffer<Matrix<float,29,1>> q_log;
    boost::circular_buffer<Matrix<float,29,1>> qd_log;
    boost::circular_buffer<Matrix<float,29,1>> tau_log;
    boost::circular_buffer<Matrix<float,29,1>> tau_GC_log;
    boost::circular_buffer<Matrix<float,29,1>> tau_Mom_log;
    boost::circular_buffer<Matrix<float,29,1>> tau_MinEff_log;

#ifndef NO_IMU
#if __XENO__
    //rt_imu_3DM_GX3_25  * imu;
    nrt_imu_3DM_GX3_25  * imu;
#else
    nrt_imu_3DM_GX3_25  * imu;
#endif
#endif
    float acc[3], angRate[3], mag[3], orientMat[9], quat[4], euler[3];

    float k[11];

public:

    Boards_ctrl_wbmc(const char * config);
    virtual ~Boards_ctrl_wbmc();

    virtual void th_init(void *);
    virtual void th_loop(void *);
    virtual void homing(void);
    int user_input(uint8_t &cmd);

    int parallel_calc_RO(void *buffer, ssize_t buff_size);
    int parallel_calc_WR(void *buffer, ssize_t buff_size);

    void control_loop();
    void control_loop_TEST();

};


#endif
