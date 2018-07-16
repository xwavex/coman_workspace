/*
   Boards_ctrl_ext.cpp

   Copyright (C) 2012 Italian Institute of Technology

   Developer:
       Alessio Margan (2013-, alessio.margan@iit.it)

*/

/**
 *
 * @author Alessio Margan (2013-, alessio.margan@iit.it)
*/

#include <iostream>
#include <boost/format.hpp>
using boost::format;


#include <Boards_ctrl_ext.h>
#include <Boards_exception.h>
#include <utils.h>

/**
 * Boards_ctrl_ext constructor
 *
 * rt  : open/bind xddp socket aka 
 * cross.domain.datagram.protocol 
 *  
 * nrt : create/open fifo 
 *  
 * used to exchange data between rt <--> nrt threads without 
 * switching mode 
 *  
 * label or name of the channels are 'boards_console' and 
 * 'boards_bc_data' 
 *  
 * @param config    yaml configuration file
 *
 */

Boards_ctrl_ext::Boards_ctrl_ext(const char * config): Boards_ctrl(config) {

    console = new Read_XDDP_pipe(std::string("boards_console"), 1024);
    xddp_bc_data = new Write_XDDP_pipe(std::string("boards_bc_data"), 16384);
    xddp_user_data = new Write_XDDP_pipe(std::string("boards_user_data"), 4096);
}

/**
 * Boards_ctrl_ext deconstructor
 *
 * stop mc boards controller 
 *  
 * stop bc data 
 *  
 * close rt <--> nrt channels
 *
 */
Boards_ctrl_ext::~Boards_ctrl_ext() {


    std::cout << "~" << typeid(this).name() << std::endl;

    std::cout << "Stop DSPs" << std::endl;
    start_stop_control(false);
    stop_rx_udp();
    start_stop_bc_boards(false);

    delete console;
    delete xddp_bc_data;
    delete xddp_user_data;
}

void Boards_ctrl_ext::init() {

    int bId = 0;

    Boards_ctrl::init();

    DPRINTF("Scan for active boards ....\n");
    int numActive = scan4active();
    sleep(1);
    DPRINTF("Found %d boards\n", numActive);

    // stop motor controllers [udp]
    start_stop_control(false);
    // try to stop broadcast if still actice [tcp]
    start_stop_bc_boards(false);
    // use config.yaml settings to set boards parameters [tcp]
    configure_boards();
    // tell to ALL dps to start broadcast data [tcp]
    start_stop_bc_boards(true);
    // warm up rx_udp thread ...
    sleep(1);

}

/**
 * use actual joints position and set as 'home position' and 
 * move towards it 
 * 
 */
void Boards_ctrl_ext::homing(void) {

    int         dummy, bId = 0;
    std::vector<float> pos(MAX_DSP_BOARDS);
    std::vector<float> vel(MAX_DSP_BOARDS);

    get_bc_data(_ts_bc_data);

    for (auto it = _mcs.begin(); it != _mcs.end(); it++) {
        bId = it->first;
        pos[bId-1] = mRAD2DEG(_ts_bc_data[bId-1].raw_bc_data.mc_bc_data.Position);
        vel[bId-1] = 25;
        it->second->get_PID(POSITION_GAINS, _stiff[bId-1], dummy, _damp[bId-1]);
        //std::cout << pos[bId-1] << std::endl;
    }
        
    homing(pos, vel);
}

/**
 * set 'home position' and move towards it
 *  
 * @param pos_deg 
 * @param vel_deg_s 
 */ 
void Boards_ctrl_ext::homing(const std::vector<float> &pos_deg, const std::vector<float> &vel_deg_s) {

    int bId = 0;

    if ( pos_deg.size() < MAX_MC_BOARDS) {
        throw(std::runtime_error(std::string("wrong size pos_deg")));
    }
    if ( vel_deg_s.size() < MAX_MC_BOARDS) {
        throw(std::runtime_error(std::string("wrong size vel_deg_s")));
    }

    for (auto it = _mcs.begin(); it != _mcs.end(); it++) {
        bId = it->first;
        _pos[bId-1]  = DEG2mRAD(pos_deg[bId-1]);
        _home[bId-1] = DEG2mRAD(pos_deg[bId-1]);
        _vel[bId-1]  = DEG2RAD(vel_deg_s[bId-1])*1000;
    }

    move();
}


int Boards_ctrl_ext::check_mcs_faults() {

    ts_bc_data_t    * ptr_bc_data = _ts_bc_data;
    uint8_t         faults;
    std::string     except_msg;

    for (auto it = _mcs.begin(); it != _mcs.end(); it++) {
        faults = (ptr_bc_data + it->first-1)->raw_bc_data.mc_bc_data.faults(); 
        // more likely you will get a TEMPERATURE fault ( ucontroller temp >70 degree ... no cooling stuff)
        // the dsp has already stop the controller .....
        // I can only notify it and you could just drink a cold beer !!! 
        if (faults) {
            except_msg = str(format("%1% board ID %2% : %3%") % __FUNCTION__ % it->first % (int)faults);
            // at least will be caugth in Thread_hook::{nrt,rt}_th_helper 
            throw(boards_error(except_msg));
        }
    }
    
    return faults;
}
    

void Boards_ctrl_ext::sense(void) {

    //bc_data_map_t   bc_map;

    //get_sync_data((void*)0);
    get_bc_data(_ts_bc_data);

    // idx --> bId-1
    //(*boards_ctrl)[0]->
    //_ts_bc_data[0].mc_bc_data.Position;

    check_mcs_faults();

    // write to pipe or cross domain socket ALL bc data
    //ssize_t nbytes = write(fd_data, (void*)&_ts_bc_data, sizeof(_ts_bc_data));
    // using (rt)xddp or (nrt)fifo could raise different errors 
    // !!!! if no one is reading got error and also other rtpipes do not works !!!!
    // SOLVED with setsockopt XDDP_POOLSZ   
    // (rt) errno ENOMEM 12 --> Cannot allocate memory ...
    // (nrt)errno        11 --> Resource temporarily unavailable

    ssize_t nbytes = xddp_bc_data->write((void*)&_ts_bc_data, sizeof(_ts_bc_data));
    if (nbytes <= 0 && errno != 11 && errno != 12) { DPRINTF("%d ", errno); perror(">> write to xddp/pipe fail"); }
    
}

int Boards_ctrl_ext::user_input(void *buffer, ssize_t buff_size) {

    int nbytes = console->read(buffer,buff_size);
    if (nbytes > 0) {
        DPRINTF("read %d bytes from xddp/pipe\n", nbytes);
    }
    return nbytes; 
}

/**
 * shortcut for setting  position/velocity/torque references 
 * using member vector _pos/_vel/_tor 
 *  
 */ 
void Boards_ctrl_ext::move(int move_mask) {

    int bId;

    //DPRINTF("move_mask %d\n", move_mask);

    if ( move_mask & (MV_POS|MV_VEL) ) {
        set_position_velocity(_pos, _vel, MAX_DSP_BOARDS);
    } else {
        if ( move_mask & MV_POS ) {
            set_position(_pos, sizeof(_pos));
        }
        if ( move_mask & MV_VEL ) {
            set_velocity(_vel, sizeof(_vel));
        }
    }

    if (move_mask & MV_TOR ) {
        set_torque(_tor, sizeof(_tor));
    }

    if ( move_mask & MV_STF) {
        set_stiffness_damping(_stiff, _damp, MAX_DSP_BOARDS);
    }

    // when impedance control is active stiff and damp are P D gains
    for (auto it = _mcs.begin(); it != _mcs.end(); it++) {
        bId = it->first;
        _user_pid_gains[bId-1].gain_set = POSITION_GAINS;
        _user_pid_gains[bId-1].p = _stiff[bId-1];
        _user_pid_gains[bId-1].d = _damp[bId-1];
    }

    ssize_t nbytes = xddp_user_data->write((void*)&_user_pid_gains, sizeof(_user_pid_gains));
    if (nbytes <= 0 && errno != 11 && errno != 12) { DPRINTF("%d ", errno); perror(">> write to xddp/pipe fail"); }


}

