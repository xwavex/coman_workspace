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

#ifndef __ECAT_MASTER_H__
#define __ECAT_MASTER_H__

#include <pthread.h>
#include <definitions.h>
#include <ecrt.h>


class ECat_master {

public:
    ECat_master(const ec_pdo_entry_reg_t *, uint32_t cycle_us);
    virtual ~ECat_master();

    int init(void);
    int config(void);

    virtual int     config_PDOs(void) = 0;
    // not define has pure virtual .... 
    virtual void    ec_hook_loop(void *) {};

protected:

    void check_master_state(void);
    void check_domain_state(void);
    void sync_distributed_clocks(void);
    static void * ec_loop(void *);

    uint32_t    ec_loop_cycle_us;

    ec_master_t *   master;
    uint8_t *       domain_pd;

    const ec_pdo_entry_reg_t * domain_regs;

private:

    pthread_t   ec_loop_thread;
    uint8_t     run;

    ec_master_state_t    master_state;

    ec_domain_t *       domain;
    ec_domain_state_t   domain_state;

};



#endif
