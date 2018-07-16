/*
   Boards_ctrl_walk.cpp

   Copyright (C) 2012 Italian Institute of Technology

   Developer:
       Alessio Margan (2013-, alessio.margan@iit.it)

*/

/**
 *
 * @author Alessio Margan (2013-, alessio.margan@iit.it)
*/

#include <Boards_ctrl_walk.h>

// DEMO_TYPE=2
// functions for test joints
extern void test_joint(std::vector<float> homingPos, int size, double freq, int r_pos[]);
extern void JointTestState(char cmd);

// DEMO_TYPE=1
// functions for walking
extern void RTControl(double RTtime, float FTSensor[],std::vector<float> homePos, int size,int *pos);
extern void InitializeWalkState();
extern void SetInitialFlag();
extern void KeyBoardControl(char cmd);


// boards ID
std::vector<int> r_leg = {  4,  6,  7,  8,  9, 10};
std::vector<int> l_leg = {  5, 11, 12, 13, 14, 15};
std::vector<int> waist = {  1,  2, 3};
std::vector<int> r_arm = { 16, 17, 18 ,19};
std::vector<int> l_arm = { 20, 21, 22, 23};
std::vector<int> neck  = { }; //{ 24, 25};

static const std::vector<float> homeVel(25,25);


// home position in degree
static const std::vector<float> homePos = {
    // lower body #15
//    0, -1,  0,  0,  0,  0,  0,   0,  0,  0,  0,   0,   0,  0,  0,
    0, 6,  0,  0,  0,  0,  0,   0,  0,  0,  0,   0,   0,  0,  0,
//  1,  2,  3,  4,  5,  6,  7,   8,  9, 10,  11, 12,  13, 14, 15
    // upper body #10 right arm to left arm, last 2 are right and left neck
    0, 90,  0,  -90, 0, -90,  0, -90,  0,  0};
// 16, 17, 18,  19, 20,  21, 22,  23, 24, 25


Boards_ctrl_walk::Boards_ctrl_walk(const char * config): Boards_ctrl_ext(config) {

    name = "boards_ctrl_walk";
    period.period = {0,1000};

#ifdef __XENO__
    schedpolicy = SCHED_FIFO;
#else
    schedpolicy = SCHED_OTHER;
#endif
    priority = sched_get_priority_max(schedpolicy);
    stacksize = PTHREAD_STACK_MIN;
}
    
void Boards_ctrl_walk::th_init(void *) {

    // configure dsp and start bc data
    // NOT start motor controller
    init();
    // homing
    homing(homePos, homeVel);

#if DEMO_TYPE==2
    InitializeWalkState();
    SetInitialFlag();
#endif

    // start MotorController !!!
    start_stop_control(true);
    sleep(3);
    g_tStart = get_time_ns();
}

void Boards_ctrl_walk::th_loop(void *) { 

    uint8_t cmd;

    try { sense(); }
    catch (...) { }

    user_input(cmd);

#if DEMO_TYPE==2
    user_loop_walk();
#elif DEMO_TYPE==1
    user_loop_test_joint();
#else
    #error "DEMO_TYPE not defined !!"
#endif
}

int Boards_ctrl_walk::user_loop_walk(void) {

    static double   RTtime;
    static float    FTSensor[12];
    static char     buff[256];

    FTSensor[0]  = 0.001*float(_ts_bc_data[25].raw_bc_data.ft_bc_data.fx);
    FTSensor[1]  = 0.001*float(_ts_bc_data[25].raw_bc_data.ft_bc_data.fy);
    FTSensor[2]  = 0.001*float(_ts_bc_data[25].raw_bc_data.ft_bc_data.fz);
    FTSensor[3]  = 0.001*float(_ts_bc_data[25].raw_bc_data.ft_bc_data.tx);
    FTSensor[4]  = 0.001*float(_ts_bc_data[25].raw_bc_data.ft_bc_data.ty);
    FTSensor[5]  = 0.001*float(_ts_bc_data[25].raw_bc_data.ft_bc_data.tz);
    FTSensor[6]  = 0.001*float(_ts_bc_data[26].raw_bc_data.ft_bc_data.fx);
    FTSensor[7]  = 0.001*float(_ts_bc_data[26].raw_bc_data.ft_bc_data.fy);
    FTSensor[8]  = 0.001*float(_ts_bc_data[26].raw_bc_data.ft_bc_data.fz);
    FTSensor[9]  = 0.001*float(_ts_bc_data[26].raw_bc_data.ft_bc_data.tx);
    FTSensor[10] = 0.001*float(_ts_bc_data[26].raw_bc_data.ft_bc_data.ty);
    FTSensor[11] = 0.001*float(_ts_bc_data[26].raw_bc_data.ft_bc_data.tz);

    
    _ts_bc_data[25].raw_bc_data.ft_bc_data.sprint(buff,sizeof(buff));
    //DPRINTF("Right %s", buff);
    _ts_bc_data[26].raw_bc_data.ft_bc_data.sprint(buff,sizeof(buff));
    //DPRINTF("Left %s", buff);

    RTtime = (get_time_ns()-g_tStart)/1e9;

    //////////////// walking pattern ////////////////////////////
    RTControl(RTtime, FTSensor, homePos, homePos.size(), _pos);

    set_position(_pos, sizeof(_pos));

    return 0;
}

int Boards_ctrl_walk::user_loop_test_joint(void) {

    static double   RTtime;

    RTtime = (get_time_ns()-g_tStart)/1e9;

    if (RTtime > 3 ) {

        test_joint(homePos, homePos.size(), 0.05, _pos);

        set_position(_pos, sizeof(_pos));
    }

    return 0;
}


int Boards_ctrl_walk::user_input(uint8_t &cmd) {

    int nbytes = Boards_ctrl_ext::user_input((void*)&cmd, sizeof(cmd));

    if (nbytes <= 0) {
        return nbytes;
    }

#if DEMO_TYPE==2
    KeyBoardControl(cmd);
#elif DEMO_TYPE==1
    JointTestState(cmd);
#else
    #error "DEMO_TYPE not defined !!"
#endif

    return nbytes;
}

