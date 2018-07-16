/*
   ECat_master.h

   Copyright (C) 2013 Italian Institute of Technology

   Developer:
       Alessio Margan (2013-, alessio.margan@iit.it)
   
*/

/**
 * @defgroup ECat_master ECat_master
 *
 * @brief TODO 
 *
 * @author Alessio Margan (2012-, alessio.margan@iit.it)
*/

#ifndef __ECAT_MASTER_BECKHOFF_H__
#define __ECAT_MASTER_BECKHOFF_H__

#include <ECat_master.h>
#include <thread_util.h>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/count.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/min.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/variance.hpp>
#include <boost/accumulators/statistics/error_of.hpp>
#include <boost/accumulators/statistics/error_of_mean.hpp>

using namespace boost::accumulators;

class ECat_master_beckhoff : public Thread_hook, public ECat_master {

public:

    ECat_master_beckhoff();
    virtual ~ECat_master_beckhoff();

    // Thread_hook pure virtual to implement
    virtual void th_init(void *);
    virtual void th_loop(void *);

    // ECat_master pure virtual to implement
    virtual int     config_PDOs(void);
    virtual void    ec_hook_loop(void *);

protected:
    
private:

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

};



#endif
