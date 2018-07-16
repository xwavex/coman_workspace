/*
   Boards_ctrl_wbmc.cpp

   Copyright (C) 2012 Italian Institute of Technology

   Developer:
       Alessio Margan (2013-, alessio.margan@iit.it)

*/

/**
 *
 * @author Alessio Margan (2013-, alessio.margan@iit.it)
*/

#include <fstream>

#ifdef __XENO__
    #include <rt_ipc.h>
#endif

#include <Boards_ctrl_wbmc.h>

std::vector<float> homePos = {
    // lower body #15
    0,  0,  0,-20,-20,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
//  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15
    // upper body #10
    0, 87,  0, -3,  0,-87,  0, -3,  0,  0};
// 16, 17, 18, 19, 20, 21, 22, 23, 24, 25

// boards ID
std::vector<int> r_leg = {  4,  6,  7,  8,  9, 10};
std::vector<int> l_leg = {  5, 11, 12, 13, 14, 15};
std::vector<int> waist = {  1,  2, 3};
//std::vector<int> r_arm = { 16, 17, 18 ,19};
//std::vector<int> l_arm = { 20, 21, 22, 23};
std::vector<int> r_arm = { 16, 17};
std::vector<int> l_arm = { };
std::vector<int> neck  = { 24, 25};


template <class T>
static void log_matrix(T t, std::string log_name) {

    using namespace Eigen;
    std::ofstream log_file(log_name.c_str());
    int cnt_ln = 0;
    //for (boost::circular_buffer<Matrix<float,29,1>>::iterator it=q_log.begin(); it!=q_log.end(); it++) {
    for (boost::circular_buffer<Matrix<float,29,1>>::iterator it=t.begin(); it!=t.end(); it++) {
        log_file << ++cnt_ln << '\t' << (*it).transpose() << std::endl;
    }
    log_file << std::flush;
    log_file.close();

}


Boards_ctrl_wbmc::Boards_ctrl_wbmc(const char * config): Boards_ctrl_ext(config) {

    name = "boards_ctrl_wbmc";
    period.period = {0,1000};

#ifdef __XENO__
    schedpolicy = SCHED_FIFO;
#else
    schedpolicy = SCHED_OTHER;
#endif
    priority = sched_get_priority_max(schedpolicy);
    stacksize = -1;

    std::string pipe;
    pipes_name[0] = std::string("parallel_calc_res");
    pipes_name[1] = std::string("parallel_calc_arg");
#ifdef __XENO__
    fd_calc_read_res  = xddp_bind(pipes_name[0].c_str());
    fd_calc_write_arg = xddp_bind(pipes_name[1].c_str());
#else
    pipe = pipe_prefix + pipes_name[0];
    mkfifo(pipe.c_str(), S_IRWXU|S_IRWXG);
    fd_calc_read_res = open(pipe.c_str(), O_RDWR);

    pipe = pipe_prefix + pipes_name[1];
    mkfifo(pipe.c_str(), S_IRWXU|S_IRWXG);
    fd_calc_write_arg = open(pipe.c_str(), O_RDWR | O_NONBLOCK);
#endif
    assert(fd_calc_read_res && fd_calc_write_arg);

    q_log.set_capacity(LOG_SIZE);
    qd_log.set_capacity(LOG_SIZE);
    tau_log.set_capacity(LOG_SIZE);
    tau_GC_log.set_capacity(LOG_SIZE);
    tau_Mom_log.set_capacity(LOG_SIZE);
    tau_MinEff_log.set_capacity(LOG_SIZE);

}

Boards_ctrl_wbmc::~Boards_ctrl_wbmc() {

#ifndef NO_IMU
    delete imu;
#endif
    close(fd_calc_read_res);
    close(fd_calc_write_arg);

#ifndef __XENO__
    std::string pipe = pipe_prefix + pipes_name[0];
    unlink(pipe.c_str());
    pipe = pipe_prefix + pipes_name[1];
    unlink(pipe.c_str());
#endif

    std::cout << "Count loop time : " << (uint64_t)count(loop_time) << std::endl;
    std::cout << "Mean loop time  ns : " << (uint64_t)mean(loop_time) << std::endl;
    std::cout << "Min loop time ns : " << min(loop_time) << std::endl;
    std::cout << "Max loop time ns : " << max(loop_time) << std::endl;
    std::cout << "Var loop time ns : " << variance(loop_time) << std::endl;
    std::cout << "Mean error loop time ns : " << boost::accumulators::error_of<tag::mean>(loop_time) << std::endl;

    log_matrix(q_log, std::string("/tmp/q_log.txt"));
    log_matrix(qd_log, std::string("/tmp/qd_log.txt"));
    log_matrix(tau_log, std::string("/tmp/tau_log.txt"));
    log_matrix(tau_GC_log, std::string("/tmp/tau_GC.txt"));
    log_matrix(tau_Mom_log, std::string("/tmp/tau_Mom.txt"));
    log_matrix(tau_MinEff_log, std::string("/tmp/tau_MinEff.txt"));

}

void Boards_ctrl_wbmc::th_init(void *) {

    k[0] = 0.9;
    k[1] = 0;
    k[4] = 0.04;
    //k[5] = 5;
    //k[6] = 0.5;

/*          0               // 0- dummy_matlab
            0,//-75,        // 1- MomJ
            0,//-0.075,     // 2- LMomCOM,
            0,//-0.25,      // 3- AMomCOM,
            0,//0.02,       // 4- MinEff,
            0,//5,          // 5- jLimP,
            0,//0.5,        // 6- jLimD,
            0,          // 7- RHandP,
            0,          // 8- RHandD,
            0,          // 9- LHandP
            0};	        // 10- LHandD
*/


    int imu_cmd =
                //0;
                CMD_EULER;
                //CMD_ACCEL_ANGRATE;
                //CMD_QUATERNION;
                //CMD_ACCEL_ANGRATE_MAG_ORIENT;

    // configure dsp and start bc data
    // NOT start motor controller
    init();
    homing();
    test();

    // read stiffness and dampping
    int dummy;
    uint8_t  bId;
    for (auto it = _mcs.begin(); it != _mcs.end(); it++) {
        bId = it->first;
        it->second->get_PID(POSITION_GAINS, _stiff[bId-1], dummy, _damp[bId-1]);
    }

#ifndef NO_IMU
#ifdef __XENO__
    //imu =  new rt_imu_3DM_GX3_25("rtser0", 115200, imu_cmd);
    imu = new nrt_imu_3DM_GX3_25("/dev/ttyACM0", 115200, imu_cmd);
#else
    imu = new nrt_imu_3DM_GX3_25("/dev/ttyACM0", 115200, imu_cmd);
#endif
#endif

    g_tStart = get_time_ns();
}

void Boards_ctrl_wbmc::th_loop(void *) {

    uint8_t cmd;

    sense();
#ifndef NO_IMU
    //imu->get_raw_data_sync(raw_data);
    imu->get_Euler(euler, 0);
#endif
    user_input(cmd);

    tNow = get_time_ns();
    //
    control_loop();
    //
    dt = get_time_ns()-tNow;
    loop_time(dt);
    //DPRINTF("oo0/-\\0oo control_loop %.6f\n", (float)dt/1e6);

    //memset((void*)_tor, 0, sizeof(_tor));
    set_torque(_tor, sizeof(_tor));
    //_pos[13] = _ts_bc_data[8].raw_bc_data.mc_bc_data.Position;
    //_pos[14] = _ts_bc_data[9].raw_bc_data.mc_bc_data.Position;
    //set_position(_pos, sizeof(_pos));

}

void Boards_ctrl_wbmc::homing(void) {

    int         bId = 0;

    get_bc_data(_ts_bc_data);

    for (auto it = _mcs.begin(); it != _mcs.end(); it++) {
        bId = it->first;
        _pos[bId-1] = DEG2mRAD(homePos[bId-1]);
        _home[bId-1] = _pos[bId-1];
        _vel[bId-1] = DEG2RAD(25)*1000;
    }

    move();
}

int Boards_ctrl_wbmc::parallel_calc_RO(void *buffer, ssize_t buff_size) {

    int     nbytes;
    /////////////////////////////////////////////////////////
    // BLOCKING read from pipe or cross domain socket
#if __XENO__
    nbytes = recvfrom(fd_calc_read_res, buffer, buff_size, 0, NULL, 0);
#else
    // BLOCKING
    nbytes = read(fd_calc_read_res, buffer, buff_size);
#endif

    if (nbytes < 0) {
        ;//perror("RO ");
    }

    return nbytes;
}

int Boards_ctrl_wbmc::parallel_calc_WR(void *buffer, ssize_t buff_size) {

    int     nbytes;
    /////////////////////////////////////////////////////////
    // NON-BLOCKING read from pipe or cross domain socket
#if __XENO__
    nbytes = sendto(fd_calc_write_arg, buffer, buff_size, MSG_DONTWAIT, NULL, 0);
#else
    // NON-BLOCKING
    nbytes = write(fd_calc_write_arg, buffer, buff_size);
#endif

    if (nbytes < 0) perror("WR ");

    return nbytes;
}

int Boards_ctrl_wbmc::user_input(uint8_t &cmd) {

    static int toggle = 1;
    McBoard * b;

    int nbytes = Boards_ctrl_ext::user_input((void*)&cmd, sizeof(cmd));

    if (nbytes <= 0) {
        return nbytes;
    }

    switch (cmd) {

    case '1':
        //use impedance control
        DPRINTF("Start POSITION control ...\n");
        start_stop_control(true);
        //start_stop_set_control(l_arm, true);
        //start_stop_set_control(r_arm, true);
        break;

    case 'H':
        DPRINTF("Set home pos\n");
        homing();
        break;

    case 'Z':
        DPRINTF("Set zero pos\n");
        for(int i=0; i<homePos.size(); i++) {
            _pos[i] = DEG2mRAD(homePos[i]);
        }
        set_position(_pos, sizeof(_pos));
        break;

    case 'A':
        DPRINTF("Set pos ref to median point of range pos\n");
        for (auto it = _mcs.begin(); it != _mcs.end(); it++) {
            b = it->second;
            _pos[b->bId-1] = (b->_max_pos + b->_min_pos) / 2;
        }
        set_position(_pos, sizeof(_pos));
        break;

    case 'S':
        for (auto it = _mcs.begin(); it != _mcs.end(); it++) {
            b = it->second;
            _damp[b->bId-1] += 10;
            _stiff[b->bId-1] = 0;
            DPRINTF("%d ", _damp[b->bId-1]);
        }
        DPRINTF("\n");
        set_stiffness_damping(_stiff, _damp, MAX_DSP_BOARDS);
        break;
    case 's':
        for (auto it = _mcs.begin(); it != _mcs.end(); it++) {
            b = it->second;
            _damp[b->bId-1] = _damp[b->bId-1] > 10 ? _damp[b->bId-1]-10 : 0;
            _stiff[b->bId-1] = 0;
            DPRINTF("%d ", _damp[b->bId-1]);
        }
        DPRINTF("\n");
        set_stiffness_damping(_stiff, _damp, MAX_DSP_BOARDS);
        break;

    case 'K':
        k[4] += 0.005;
        break;
    case 'k':
        k[4] -= 0.005;
        break;
    case 'M':
        k[1] = -30;
        break;

    default:
        DPRINTF("Stop control ...\n");
        start_stop_control(false);
        clear_mcs_faults();
        break;
    }

    return nbytes;
}

