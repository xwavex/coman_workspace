#include <ECat_master.h>

#include <bits/local_lim.h>
#include <assert.h>

#include <utils.h>



ECat_master::ECat_master(const ec_pdo_entry_reg_t * _regs, uint32_t cycle_us) {

    domain_regs = _regs;
    ec_loop_cycle_us = cycle_us;

}

int ECat_master::config(void) {

    printf("request master\n");
    master = ecrt_request_master(0);
    assert(master);

    domain = ecrt_master_create_domain(master);
    assert(domain);

    printf("Configuring PDOs...\n");
    if ( config_PDOs() ) {
        assert(0);
    }

    if (ecrt_domain_reg_pdo_entry_list(domain, domain_regs)) {
        fprintf(stderr, "PDO entry registration failed!\n");
        assert(0);
    }

    printf("Activating master...\n");
    if (ecrt_master_activate(master))
        assert(0);

    if (!(domain_pd = ecrt_domain_data(domain))) {
        assert(0);
    }
    fprintf(stderr, "domain1_pd:  0x%.6x\n", (unsigned int)(*domain_pd));

    return 0;
}

ECat_master::~ECat_master() {

    run = 0;
    pthread_join(ec_loop_thread, NULL);

    printf("release master\n");
    ecrt_release_master(master);
}

int ECat_master::init(void) {

    pthread_attr_t      attr;
    int                 policy;
    cpu_set_t           cpu_set;
    struct sched_param  schedparam;

    // ethercat init stuff 
    config();

#ifdef __XENO__
    policy = SCHED_FIFO;
#else
    //policy = SCHED_FIFO;
    policy = SCHED_OTHER;
#endif

    run = 1;
    // thread configuration and creation
    pthread_attr_init(&attr);
    pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(&attr, policy);
    schedparam.sched_priority = sched_get_priority_max(policy);
    pthread_attr_setschedparam(&attr, &schedparam);
    pthread_attr_setstacksize(&attr, PTHREAD_STACK_MIN);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
    CPU_SET(2,&cpu_set);
    pthread_attr_setaffinity_np(&attr, sizeof(cpu_set), &cpu_set);
    if ( pthread_create(&ec_loop_thread, &attr, ec_loop, (void*)this) ) {
        perror("pthread_create fail ");
        exit(1);
    }
    pthread_attr_destroy(&attr);

    return 0;

}


void ECat_master::check_domain_state(void)
{
    ec_domain_state_t ds = {};

	ecrt_domain_state(domain, &ds);

    if (ds.working_counter != domain_state.working_counter) {
        DPRINTF("Domain1: WC %u.\n", ds.working_counter);
    }

    if (ds.wc_state != domain_state.wc_state) {
        DPRINTF("Domain1: State %u.\n", ds.wc_state);
    }

    domain_state = ds;
}


void ECat_master::check_master_state(void)
{
    ec_master_state_t ms;

	ecrt_master_state(master, &ms);

    if (ms.slaves_responding != master_state.slaves_responding) {
        DPRINTF("%u slave(s).\n", ms.slaves_responding);
    }

    if (ms.al_states != master_state.al_states) {
        DPRINTF("AL states: 0x%02X.\n", ms.al_states);
    }

    if (ms.link_up != master_state.link_up) {
        DPRINTF("Link is %s.\n", ms.link_up ? "up" : "down");
    }

    master_state = ms;
}


void ECat_master::sync_distributed_clocks(void)
{
    uint64_t dc_time_ns;
#if SYNC_MASTER_TO_REF
    uint32_t ref_time = 0;
    uint64_t prev_app_time = dc_time_ns;
#endif

    dc_time_ns = get_time_ns();

    // set master time in nano-seconds
    ecrt_master_application_time(master, dc_time_ns);

#if SYNC_MASTER_TO_REF
    // get reference clock time to synchronize master cycle
    ecrt_master_reference_clock_time(master, &ref_time);
    dc_diff_ns = (uint32_t) prev_app_time - ref_time;
#else
    // sync reference clock to master
    ecrt_master_sync_reference_clock(master);
#endif

    // call to sync slaves to ref slave
    ecrt_master_sync_slave_clocks(master);
}


void * ECat_master::ec_loop(void * _) {

    ECat_master     * kls = (ECat_master*)_;

    struct timespec starttp, periodtp;
    unsigned long overruns; 
    int cycle_counter = 0;
    int ret;
    const char th_name[] = "ec_loop";

    periodtp.tv_sec = 0ULL;
    periodtp.tv_nsec = kls->ec_loop_cycle_us * 1000ULL; 
    clock_gettime(CLOCK_REALTIME, &starttp);
    starttp.tv_nsec += periodtp.tv_nsec;
    tsnorm(&starttp);

#ifdef __XENO__
    pthread_set_mode_np(0, PTHREAD_WARNSW);
    pthread_set_name_np(pthread_self(), th_name);

    ret = pthread_make_periodic_np(pthread_self(), &starttp, &periodtp);
    if ( ret != 0 ) {
        DPRINTF("%s : pthread_make_periodic_np() return code %d\n", th_name, ret);
        exit(1);
    }
#else
    clock_gettime(CLOCK_MONOTONIC ,&starttp);
#endif

    while (kls->run) {

#ifdef __XENO__
        ret = pthread_wait_np(&overruns);
        switch (ret) {
            case ETIMEDOUT :
                DPRINTF("%s : pthread_wait_np() ETIMEDOUT %lu\n", th_name, overruns);
                break;
            case EPERM :
            case EWOULDBLOCK :
            case EINTR :
                DPRINTF("%s : pthread_wait_np() return code %d\n", th_name, ret);
                exit(1);
        }
#else
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &starttp, NULL);
        starttp.tv_nsec += periodtp.tv_nsec;
        tsnorm(&starttp);
#endif

        // receive EtherCAT
        ecrt_master_receive(kls->master);
        ecrt_domain_process(kls->domain);

        kls->check_domain_state();

        if (!(cycle_counter % 1000)) {
			kls->check_master_state();
		}

        // 
        kls->ec_hook_loop(0);

        //sync DC
        //rt_sync();
        // sync distributed clock just before master_send to set
        // most accurate master clock time
        //kls->sync_distributed_clocks();

        // send process data
        ecrt_domain_queue(kls->domain);
        ecrt_master_send(kls->master);  

    }

    return 0;


}

