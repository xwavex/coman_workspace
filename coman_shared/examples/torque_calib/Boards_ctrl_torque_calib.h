
#include <utils.h>
#include <thread_util.h>

#include <Boards_ctrl_ext.h>

#define HOMING 0
#define DO_STOP_CONTROL 1
#define DO_LOAD_PARAM 2
#define WAIT_AFTER_LOAD_PARAM 3
#define DO_SET_OFFSET 4
#define DO_SAVE_PARAM 5
#define DO_PRINT 6
#define DO_NOTHING 99


///////////////////////////////////////////////////////////////////////////////
// Globals
///////////////////////////////////////////////////////////////////////////////

// home velocity in degree/s
static const std::vector<float> homeVel(25,25);

// home position in degree
static const std::vector<float> homePos = {
    // lower body #15
    0,  0,  0,  0,  0,  0,  0,  0, 30,  0,  0,  0,  0, 30,  0,
//  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15
    // upper body #10
    0, 87,  0, -3,  0,-87,  0, -3,  0,  0};
// 16, 17, 18, 19, 20, 21, 22, 23, 24, 25

std::vector<int> r_leg = {  4,  6,  7,  8,  9, 10};
std::vector<int> l_leg = {  5, 11, 12, 13, 14, 15};
std::vector<int> waist = {  1,  2, 3};
std::vector<int> r_arm = { 16, 17, 18 ,19};
std::vector<int> l_arm = { 20, 21, 22, 23};
std::vector<int> neck  = { /*24, 25*/ };

/*
using namespace boost::accumulators;
typedef accumulator_set<int,
features<
tag::count
,tag::mean
,tag::min
,tag::max
,tag::variance(lazy)
>
> torque_sample_t;
std::vector<torque_sample_t> torque_samples(25+1);
torque_sample_t test_torque;
*/

///////////////////////////////////////////////////////////////////////////////
//
///////////////////////////////////////////////////////////////////////////////


class Calib_thread : public Thread_hook, public Boards_ctrl_ext {

    int state;
    unsigned long long  tNow, dt, g_tStart;
/*
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
*/

public:

    Calib_thread(const char * config): Boards_ctrl_ext(config) {

        name = "Calib_thread";
        period.period = {0,5000};

        schedpolicy = SCHED_OTHER;
        priority = sched_get_priority_max(schedpolicy)/2;
        stacksize = PTHREAD_STACK_MIN;
    }

    ~Calib_thread() {

/*
        std::cout << "Count loop time : " << (uint64_t)count(loop_time) << std::endl;
        std::cout << "Mean loop time  ns : " << (uint64_t)mean(loop_time) << std::endl;
        std::cout << "Min loop time ns : " << min(loop_time) << std::endl;
        std::cout << "Max loop time ns : " << max(loop_time) << std::endl;
        std::cout << "Var loop time ns : " << variance(loop_time) << std::endl;
        std::cout << "Mean error loop time ns : " << boost::accumulators::error_of<tag::mean>(loop_time) << std::endl;
*/
    }

    virtual void th_init(void *) {

        // configure dsp and start bc data
        // NOT start motor controller
        init();
        // read current position and set as homing
        homing();
        test();

        state = DO_NOTHING;
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

        static int toggle = 1;
        McBoard * b;

        int nbytes = Boards_ctrl_ext::user_input((void*)&cmd, sizeof(cmd));

        if (nbytes <= 0) {
            return nbytes;
        }


        switch (cmd) {
        case '1':
            start_stop_control(true);
            break;
        case '2':
            homing(homePos, homeVel);
            break;
        case 'S':
            state = HOMING;
            g_tStart = get_time_ns();
            break;
        case 'P':
            state = DO_PRINT;
            break;

        default:
            DPRINTF("Stop control ...\n");
            start_stop_control(false);
            clear_mcs_faults();
            state = DO_NOTHING;
            break;
        }



        return nbytes;
    }



    void user_loop(void *) {

        McBoard * b;
        int bId;

        uint64_t now = get_time_ns();

        switch (state) {

            case HOMING :
                // home reached
                if ( (now - g_tStart) > 5 * 1e9 ) {
                    state = DO_STOP_CONTROL;
                    DPRINTF("%d HOMING -> %d DO_STOP_CONTROL\n", HOMING, DO_STOP_CONTROL);
                }
                break;

            case DO_STOP_CONTROL :
                start_stop_control(false);
                state = DO_LOAD_PARAM;
                DPRINTF("%d DO_STOP_CONTROL -> %d DO_LOAD_PARAM\n", DO_STOP_CONTROL, DO_LOAD_PARAM);
                break;

            case DO_LOAD_PARAM :
                for (mcs_map_t::iterator it = _mcs.begin(); it != _mcs.end(); it++) {
                    b = it->second;
                    b->setItem(LOAD_PARAMS_FROM_FLASH, NULL, 0);
                }
#if 0
                for (int bId=1; bId<=MC_BOARDS_NUM ; bId++) {
                    if ( (*boards_ctrl)[bId] ) {
                        (*boards_ctrl)[bId]->getItem(GET_TORQUE_FACTORS, NULL, 0, REPLY_TORQUE_FACTORS, &torque_factor, sizeof(torque_factor));
                        torque_factor.offset = 0;
                        (*boards_ctrl)[bId]->setItem(SET_TORQUE_FACTORS, &torque_factor, sizeof(torque_factor));
                    }
                }
#endif
                state = WAIT_AFTER_LOAD_PARAM;
                DPRINTF("%d DO_LOAD_PARAM -> %d WAIT_AFTER_LOAD_PARAM\n", DO_LOAD_PARAM, WAIT_AFTER_LOAD_PARAM);
                break;

            case WAIT_AFTER_LOAD_PARAM :
                if ( (now - g_tStart) > 10 * 1e9 ) {
                    state = DO_SET_OFFSET;
                    //state = 99;
                    DPRINTF("%d WAIT_AFTER_LOAD_PARAM -> %d DO_SET_OFFSET\n", WAIT_AFTER_LOAD_PARAM, DO_SET_OFFSET);
                }
                break;

            case DO_SET_OFFSET :
#if 0
                for (int bId=1; bId<=MC_BOARDS_NUM ; bId++) {
                    torque_samples[bId](bc_data[bId-1].mc_bc_data.Torque);
                }

                if ( (now - g_tStart) > 12 * 1e9 ) {
                    for (int bId=1; bId<=MC_BOARDS_NUM ; bId++) {
                        if ( (*boards_ctrl)[bId] ) {
                            DPRINTF("%d %d %d %d %d\n", bId,
                                    (short)count(torque_samples[bId]),
                                    (short)mean(torque_samples[bId]),
                                    (short)min(torque_samples[bId]),
                                    (short)max(torque_samples[bId]));

                            (*boards_ctrl)[bId]->getItem(GET_TORQUE_FACTORS, NULL, 0, REPLY_TORQUE_FACTORS, &torque_factor, sizeof(torque_factor));
                            torque_factor.offset = -(short)mean(torque_samples[bId])*1e4/torque_factor.multiplier;
                            (*boards_ctrl)[bId]->setItem(SET_TORQUE_FACTORS, &torque_factor, sizeof(torque_factor));
                        }
                    }
                    //state = DO_SAVE_PARAM;
                    state = DO_PRINT;

                }
#else
                for (mcs_map_t::iterator it = _mcs.begin(); it != _mcs.end(); it++) {
                    b = it->second;
                    b->setItem(CALIBRATE_TORQUE_OFFSET, NULL, 0);
                }

                state = DO_SAVE_PARAM;
                DPRINTF("%d DO_SET_OFFSET -> %d DO_SAVE_PARAM\n", DO_SET_OFFSET, DO_SAVE_PARAM);
#endif
                break;

            case DO_SAVE_PARAM :
                if ( (now - g_tStart) > 15 * 1e9 ) {
                    for (mcs_map_t::iterator it = _mcs.begin(); it != _mcs.end(); it++) {
                        b = it->second;
                        b->setItem(SAVE_PARAMS_TO_FLASH, NULL, 0);
                    }

                    DPRINTF("DO_SAVE_PARAM\n", DO_SAVE_PARAM);
                    state = DO_PRINT;
                }
                break;

            case DO_PRINT :
                DPRINTF("tor ");
                for (mcs_map_t::iterator it = _mcs.begin(); it != _mcs.end(); it++) {
                    bId = it->first;
                    DPRINTF("(%d)%d  ", bId, _ts_bc_data[bId-1].raw_bc_data.mc_bc_data.Torque);
                }
                DPRINTF("\n");
                break;

            default:
                break;
        }


    }
};


