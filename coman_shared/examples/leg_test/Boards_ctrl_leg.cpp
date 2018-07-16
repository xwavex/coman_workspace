/*
   Boards_ctrl_leg.cpp

   Copyright (C) 2012 Italian Institute of Technology

   Developer:
       Alessio Margan (2013-, alessio.margan@iit.it)
   
*/

/**
 *
 * @author Alessio Margan (2013-, alessio.margan@iit.it)
*/

#include <Boards_ctrl_leg.h>

double torque_trj_scale = 0.1;

Boards_ctrl_leg::Boards_ctrl_leg(const char * config): Boards_ctrl_ext(config) {

    name = "boards_ctrl_leg";
    period.period = {0,1000}; 

    schedpolicy = SCHED_FIFO;
    priority = sched_get_priority_max(schedpolicy);
    stacksize = PTHREAD_STACK_MIN;

}

///////////////////////////////////////////////////////////////////////////////////
//
///////////////////////////////////////////////////////////////////////////////////

void Boards_ctrl_leg::th_init(void *) {

    int bId, dummy;

    init();
    
    for (mcs_map_t::iterator it = _mcs.begin(); it != _mcs.end(); it++) {
        bId = it->first;
        it->second->get_PID(POSITION_GAINS, _stiff[bId-1], dummy, _damp[bId-1]);
    }
    
}

void Boards_ctrl_leg::th_loop(void *) { 

    /////////////////////////////////////////////////////////
    /// Sense
    sense();
    /////////////////////////////////////////////////////////
    
    uint8_t cmd;
    user_input(cmd);

    /////////////////////////////////////////////////////////
    /// compute new references ..... !!!!!!!!!
    /////////////////////////////////////////////////////////

    /////////////////////////////////////////////////////////
    /// Move
    move();
    impedance_ctrl();
    /////////////////////////////////////////////////////////

}

///////////////////////////////////////////////////////////////////////////////////
//
///////////////////////////////////////////////////////////////////////////////////

int Boards_ctrl_leg::user_input(uint8_t &cmd) {

    static uint8_t pid_flag;
    int nbytes;
    //uint8_t cmd;
    int tmp;

    nbytes = Boards_ctrl_ext::user_input((void*)&cmd, sizeof(cmd));

    if (nbytes <= 0) {
        return nbytes;
    }

    // process char
    DPRINTF("RX %c\n", cmd);
    switch ( cmd ) {
        case 'a':
            _pos[0] += 10000;
            break;
        case 'z':
            _pos[0] -= 10000;
            break;
        case 'A':
            _pos[0] += 50000;
            break;
        case 'Z':
            _pos[0] -= 50000;
            break;

            // tendon motor 
        case 's':
            _pos[1] += 100000;
            break;
        case 'x':
            _pos[1] -= 100000;
            break;
        case 'S':
            _pos[1] += 1000000;
            break;
        case 'X':
            _pos[1] -= 1000000;
            break;

        case 'F':
            _stiff[0] += 10000;
            DPRINTF("stiff %d\n", _stiff[0]);
            break;
        case 'f':
            _stiff[0] -= 10000;
            DPRINTF("stiff %d\n", _stiff[0]);
            break;
        case 'V':
            _damp[0] += 200;
            break;
        case 'v':
            _damp[0] -= 200;
            break;

        case 't':
            homing();
            g_tStart = get_time_ns();
            _trj_flag = 1;
            break;
        case 'y':
            _trj_flag = 0;
            break;

#if 1
            // calcio
        case 'K':
            _pos[0] = 220000;
            break;
        case 'k':
            _pos[0] = 400000;
            break;
        case '5':
            torque_trj_scale -= 0.1;
            break;
        case '6':
            torque_trj_scale += 0.1;
            break;

#else
        case 'l':
            set_abs_offset(0);
            break;
        case 'k':
            set_abs_offset(20);
            break;
        case 'm':
            set_abs_offset(-20);
            break;
        case 'j':
            set_abs_offset(1);
            break;
        case 'n':
            set_abs_offset(-1);
            break;
        case 'q':
            set_abs_offset(1165);
            break;
#endif

        case '1':
            DPRINTF("Start control ...\n");
            start_stop_control(true);
            //start_stop_single_control(1, true);
            //start_stop_single_control(2, true, POSITION_MOVE);
            homing();
            break;
        case '2':
            DPRINTF("Start control ...\n");
            start_stop_control(false);
            break;
        case '0':
            _mcs[1]->setItem(SAVE_PARAMS_TO_FLASH, NULL, 0);
            break;


        case 'P':
            _mcs[1]->set_PID_increment(TORQUE_GAINS, 10, 0, 0);
            //_mcs[1]->set_PID_increment(POSITION_GAINS, 1000, 0, 0);
            break;
        case 'p':
            _mcs[1]->set_PID_increment(TORQUE_GAINS, -10, 0, 0);
            //_mcs[1]->set_PID_increment(POSITION_GAINS, -1000, 0, 0);
            break;
        case 'I':
            _mcs[1]->set_PID_increment(TORQUE_GAINS, 0, 1, 0);
            //_mcs[1]->set_PID_increment(POSITION_GAINS, 0, 1, 0);
            break;
        case 'i':
            _mcs[1]->set_PID_increment(TORQUE_GAINS, 0, -1, 0);
            //_mcs[1]->set_PID_increment(POSITION_GAINS, 0, -1, 0);
            break;
        case 'D':
            //_mcs[1]->set_PID_increment(TORQUE_GAINS, 0, 1, 0);
            _mcs[1]->set_PID_increment(POSITION_GAINS, 0, 0, 50);
            break;
        case 'd':
            //_mcs[1]->set_PID_increment(TORQUE_GAINS, 0, -1, 0);
            _mcs[1]->set_PID_increment(POSITION_GAINS, 0, 0, -50);
            break;


        default:
            break;
    } 

    //if (pid_flag) { }

    return nbytes;
}


void Boards_ctrl_leg::move(void) {

    static double freq_Hz = 2;
    uint64_t tNow = get_time_ns();
    if (! g_tStart) {
        g_tStart = get_time_ns();
    }
    uint64_t dt_ns = tNow - g_tStart;
    double  trj = sin((2.0 * M_PI * freq_Hz * dt_ns)/1e9); // -1 .. 1

    //if (_trj_flag == 1) {
    //    _pos[0] = _home[0] + (20000 * trj);         
    //}

    for (int i=0; i<_mcs.size(); i++) {
        _vel[i] = DEG2RAD(600)*1000;
    }

    // check main motor leg position limit 
    if (_pos[0] > 410000) {
        _pos[0] = 410000;
    }
    if (_pos[0] < 200000) {
        _pos[0] = 200000;
    }

    set_position(_pos, sizeof(_pos));
    set_velocity(_vel, sizeof(_vel));
    //set_torque(_tor, sizeof(_tor));

}

void Boards_ctrl_leg::impedance_ctrl(void) {

    static double mass = 13.0;
    static double omega = 16.0;
    static int A = -3000;
    static int B = 0;

    static double freq_Hz = 2;
    uint64_t tNow = get_time_ns();
    if (! g_tStart) {
        g_tStart = get_time_ns();
    }
    uint64_t dt_ns = tNow - g_tStart;
    //double  trj = 1 - cos((2.0 * M_PI * freq_Hz * dt_ns)/1e9); // 0 .. 2
    double  trj = (sin((2.0 * M_PI * freq_Hz * dt_ns)/1e9)) < 0 ? -1 : 1; // -1 .. 1

    if (_trj_flag == 1) {
        // !!!!!! mNm / 10 
        //_tor[0] = 0; //5000 + (-1000 * trj);

        if (torque_trj_scale > 2) {
            torque_trj_scale = 2;
        }
        if (torque_trj_scale <= 0.1) {
            torque_trj_scale = 0.1;
        }
        // !!!!!! mNm / 10 
        _tor[0] = 2 * 100 * torque_trj_scale / ( mass * omega ) * ( A * sin(omega*dt_ns/1e9) - B * cos(omega*dt_ns/1e9) );
        _tor[0] += 400 * torque_trj_scale;
        //DPRINTF("tor %d\n ", _tor[0]);
    } else {
        memset((void*)_tor, 0, sizeof(_tor));
    }

    set_stiffness_damping(_stiff, _damp, MAX_DSP_BOARDS);
    set_torque(_tor, sizeof(_tor));

    //_mcs[1]->test_setting();
}

void Boards_ctrl_leg::set_abs_offset(int dir) {

    static uint16_t offset;

    if ( dir != 0 ) {
        offset += dir;
    } else {
        _mcs[1]->getItem(GET_ABSOLUTE_ZERO, NULL, 0, REPLY_ABSOLUTE_ZERO, &offset, sizeof(offset));
        DPRINTF("GET_ABSOLUTE_ZERO %d\n", offset);
    }

    _mcs[1]->setItem(SET_ABSOLUTE_ZERO, &offset, sizeof(offset));
    DPRINTF("SET_ABSOLUTE_ZERO %d\n", offset);

}
