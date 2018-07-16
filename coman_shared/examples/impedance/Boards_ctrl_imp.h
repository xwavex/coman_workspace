
#include <utils.h>
#include <thread_util.h>

#include <Boards_ctrl_ext.h>


///////////////////////////////////////////////////////////////////////////////
// Globals
///////////////////////////////////////////////////////////////////////////////


std::vector<int> r_leg = {  4,  6,  7,  8,  9, 10};
std::vector<int> l_leg = {  5, 11, 12, 13, 14, 15};
std::vector<int> waist = {  1,  2, 3};
std::vector<int> r_arm = { 16, 17, 18 ,19};
std::vector<int> l_arm = { 20, 21, 22, 23};
std::vector<int> neck  = { /*24, 25*/ };


///////////////////////////////////////////////////////////////////////////////
//
///////////////////////////////////////////////////////////////////////////////


class Impedance_thread : public Thread_hook, public Boards_ctrl_ext {

    unsigned long long  tNow, dt, g_tStart;
    group_ref_comp_t    stiff_damp_group;


public:

    Impedance_thread(const char * config): Boards_ctrl_ext(config) {

        name = "Impedance_thread";
        period.period = {0,1000};

    #ifdef __XENO__
        schedpolicy = SCHED_FIFO;
    #else
        schedpolicy = SCHED_OTHER;
    #endif
        priority = sched_get_priority_max(schedpolicy);
        stacksize = PTHREAD_STACK_MIN;
    }

    ~Impedance_thread() {

    }

    virtual void th_init(void *) {

        McBoard * b;
        // configure dsp and start bc data
        // NOT start motor controller
        init();
        // read current position and set as homing
        homing();

        // set stiff and damp to all mcs
        for (auto it = _mcs.begin(); it != _mcs.end(); it++) {
            _stiff[it->first-1] = 40000;
            _damp[it->first-1] = 5000;
            //_stiff[it->first-1] = 0;
            //_damp[it->first-1] = 100;
        }
        set_stiffness_damping(_stiff, _damp, MAX_MC_BOARDS);

        // set just damp to arms
        stiff_damp_group.clear();
        for (auto it = l_arm.begin(); it != l_arm.end(); it++) {
            stiff_damp_group[*it] = std::make_pair(0, 100); 
        }
        for (auto it = r_arm.begin(); it != r_arm.end(); it++) {
            stiff_damp_group[*it] = std::make_pair(0, 100); 
        }
        set_stiffness_damping_group(stiff_damp_group);

        g_tStart = get_time_ns();

    }

    virtual void th_loop(void *) {

        uint8_t cmd;
        tNow = get_time_ns();

        sense();

        user_input(cmd);

        user_loop(0);

        dt = get_time_ns()-tNow;
        //loop_time(dt);

    }

    int user_input(uint8_t &cmd) {

        int nbytes = Boards_ctrl_ext::user_input((void*)&cmd, sizeof(cmd));

        if (nbytes <= 0) {
            return nbytes;
        }

        switch (cmd) {
            case '1':
                DPRINTF("Start control ...\n");
                start_stop_control(true);
                break;

            default:
                DPRINTF("Stop control ...\n");
                start_stop_control(false);
                clear_mcs_faults();
                break;
        }

        return nbytes;
    }



    void user_loop(void *) {

        McBoard * b;
        int bId;

        // you should send a torque reference at least every 100 ms
        // it is a kind of watchdog, otherwise the DSP will stop the controller
        memset((void*)_tor, 0, sizeof(_tor));
        set_torque(_tor, sizeof(_tor));    

        uint64_t now = get_time_ns();



    }
};


