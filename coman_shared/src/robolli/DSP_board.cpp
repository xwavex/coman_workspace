/*
    DSP_board.cpp

    Copyright (C) 2012 Italian Institute of Technology

    Developer:
        Alessio Margan (2012-, alessio.margan@iit.it)

*/

#include <DSP_board.h>

#include <sys/time.h>
#include <sys/socket.h>
#include <errno.h>
#include <unistd.h>
#include <assert.h>
#include <string.h>
#include <stdio.h>

#include <iostream>
#include <fstream>
#include <boost/format.hpp>

#include <utils.h>
#include <signals.h>
#include <CommProtocol.hpp>
#include <Boards_exception.h>


#ifdef __XENO__
    #include <rtdk.h>
    #include <rtnet.h>
    #define DPRINTF rt_printf
#else
    #define DPRINTF printf
#endif


Dsp_Board::Dsp_Board(uint8_t  *replyScan4Active) {

    bType   = replyScan4Active[3];
    bId     = replyScan4Active[4];
    memset(ip_addr, 0, 16);
    sprintf(ip_addr, "%d.%d.%d.%d", replyScan4Active[8], replyScan4Active[7], replyScan4Active[6], replyScan4Active[5]);

    sock_fd = socket(AF_INET, SOCK_STREAM, 0);

    // set socket timeout
#ifdef __XENO__
    // This socket control option is used to specify the time-out on the socket before it returns.
    // It is used typically to wait for data on a Read.
    // The time-out specifies the amount of time the function will wait for data before it returns.
    int64_t timeout_ns = 250000000LL;
    if ( ioctl(sock_fd, RTNET_RTIOC_TIMEOUT, &timeout_ns) < 0 )
        DPRINTF("ioctl RTNET_RTIOC_TIMEOUT failed\n");
#else
    struct timeval timeout;
    timeout.tv_sec =  1;
    timeout.tv_usec = 0;
    if ( setsockopt (sock_fd, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout,
                     sizeof(timeout)) < 0 )
        DPRINTF("setsockopt SO_RCVTIMEO failed\n");

    if ( setsockopt (sock_fd, SOL_SOCKET, SO_SNDTIMEO, (char *)&timeout,
                     sizeof(timeout)) < 0 )
        DPRINTF("setsockopt SO_SNDTIMEO failed\n");
#endif

    sock_addr.sin_family = AF_INET;
    sock_addr.sin_addr.s_addr = inet_addr(ip_addr);
    sock_addr.sin_port = htons(23);

    if ( connect(sock_fd, (sockaddr *)&sock_addr, sizeof(sockaddr_in)) < 0 ) {
        perror("connect :");
        throw(boards_error(std::string(ip_addr)));
    }

    pthread_mutex_init(&dsp_data_mutex, NULL);
    pthread_mutex_init(&dsp_sock_mutex, NULL);

    dsp_log.set_capacity(LOG_SIZE);
    _rx_bc_prec = 0;
}


Dsp_Board::~Dsp_Board() {

    std::cout << "~" << typeid(this).name() << " bId " <<  int(bId) <<std::endl;

    print_stat();
    dump_log();
    close(sock_fd);
    pthread_mutex_destroy(&dsp_data_mutex);
    pthread_mutex_destroy(&dsp_sock_mutex);
}

/**
 * send a set command
 *
 *
 * @param reqCmd
 * @param src
 * @param srcBytes
 *
 * @return int
 */
int Dsp_Board::setItem(int reqCmd, void *src, int srcBytes)
{
    int ret;
    TCPCommPacket req(reqCmd);

    if ( srcBytes > 0 && src != NULL ) {
        req.appendData((uint8_t*)src, srcBytes);
    }

    // Send request
    pthread_mutex_lock(&dsp_sock_mutex);
    ret = req.sendToTCPSocket(sock_fd);
    pthread_mutex_unlock(&dsp_sock_mutex);

    if ( ret ) {
        DPRINTF("[TCP]{%d} Fail sendTo\n", bId);
    }

    return ret;
}

/**
 * send a request command and wait for reply
 *
 * @param reqCmd
 * @param src
 * @param srcBytes
 * @param resCmd
 * @param dst
 * @param dstBytes
 *
 * @return int
 */
int Dsp_Board::getItem(int reqCmd, void *src, int srcBytes,
                       int resCmd, void *dst, int dstBytes)
{
    int ret;
    TCPCommPacket req(reqCmd), rep(resCmd);

    if ( srcBytes > 0 && src != NULL ) {
        req.appendData((uint8_t*)src, srcBytes);
    }

    // Send request
    pthread_mutex_lock(&dsp_sock_mutex);
    ret = req.sendToTCPSocket(sock_fd);
    pthread_mutex_unlock(&dsp_sock_mutex);

    if ( ret ) {
        DPRINTF("[TCP]{%d} Fail sendTo\n", bId);
        return ret;
    }

    // Receive response
    pthread_mutex_lock(&dsp_sock_mutex);
    ret = rep.recvFromTCPSocket(sock_fd);
    pthread_mutex_unlock(&dsp_sock_mutex);

    if ( ret ) {
        DPRINTF("[TCP]{%d} Fail recvFrom reply\n", bId);
        return ret;
    }

    rep.readData((uint8_t *)dst, dstBytes);

    return 0;
}

/**
 * start/stop broadcast, set bc_rate via TCP packet
 *
 * @param start_stop    true/false;
 *
 */
void Dsp_Board::start_stop_bc(uint8_t start_stop) {

    // start_stop = true --> start bc
    // start_stop = false --> stop bc
    struct timespec ts;
    int try_count = 0;
    TCPCommPacket   bc_rate_pkt(SET_BCAST_RATE);
    uint8_t         bc_rate_cmd[] = {bc_rate, !!start_stop};

    bc_rate_pkt.appendData(bc_rate_cmd, sizeof(bc_rate_cmd));
    while ( bc_rate_pkt.sendToTCPSocket(sock_fd) ) {
        DPRINTF("[TCP]{%d} Fail send bcast rate to start/stop bc\n", bId);
        ts.tv_sec = 0;
        ts.tv_nsec = 100*1e6; // 100 ms
        clock_nanosleep(CLOCK_REALTIME, 0, &ts, NULL);
        if ( ++try_count > 10 ) {
            break;
        }
    }

    if ( start_stop ) {
        dsp_log.clear();
        _bc_tStart = get_time_ns();
    }
    stopped = !start_stop;
}

void Dsp_Board::print_me(void) {

    DPRINTF("ID %d type %d addr %s\n",
            bId,
            bType,
            ip_addr);
}

void Dsp_Board::measure_bc_freq(void)
{
    uint64_t bc_loop , tNow = get_time_ns();

    if ( _rx_bc_prec > 0 ) {
        bc_loop = tNow - _rx_bc_prec;
        bc_freq(bc_loop);
    }
    _rx_bc_prec = tNow;

}

void Dsp_Board::print_stat(void) {

    DPRINTF("ID %d %s\n", bId, ip_addr);
    DPRINTF("\t bcast freq us : min %.3f max %.3f avg %.3f ", min(bc_freq)/1e3, max(bc_freq)/1e3, mean(bc_freq)/1e3);
    DPRINTF("rx %lu\n", count(bc_freq));
}


void Dsp_Board::on_bc_data(uint8_t *raw_bc_buff) {

    if ( !stopped ) {
        measure_bc_freq();
    }

    pthread_mutex_lock(&dsp_data_mutex);
    ts_bc_data.ts_rx = get_time_ns() - _bc_tStart;
    memcpy((void*)&ts_bc_data.raw_bc_data, raw_bc_buff, get_bc_data_size());
    pthread_mutex_unlock(&dsp_data_mutex);

    if ( !stopped ) {
        dsp_log.push_back(ts_bc_data);
    }
}

void Dsp_Board::get_bc_data(ts_bc_data_t &dest_data) {

    pthread_mutex_lock(&dsp_data_mutex);
    memcpy(&dest_data, &ts_bc_data, sizeof(ts_bc_data_t));
    pthread_mutex_unlock(&dsp_data_mutex);
}



/**
 * write log file in /tmp/log_bId_<...>.txt, see LOG_SIZE define
 * for circular buffer dimension
 *
 */
void Dsp_Board::dump_log(void) {

    char buffer[1024];
    unsigned char bc_data_board_type = 0;

    std::string filename = str(boost::format("/tmp/log_bId_%1%.txt") % (int)bId);
    std::ofstream log_file(filename.c_str());

    for ( boost::circular_buffer<ts_bc_data_t>::iterator it=dsp_log.begin(); it!=dsp_log.end(); it++ ) {
        //while ( ! dsp_log.empty() ) {
        log_file << boost::format("%1%\t") % (*it).ts_rx;
        bc_data_board_type = (*it).raw_bc_data.bc_header._command;
        switch ( bc_data_board_type ) {
            case BCAST_MC_DATA_PACKETS :
                (*it).raw_bc_data.mc_bc_data.sprint(buffer, sizeof(buffer));
                break;
            case BCAST_FT_DATA_PACKETS :
                (*it).raw_bc_data.ft_bc_data.sprint(buffer, sizeof(buffer));
                break;
            default:
                DPRINTF("dump_log unknown bc_data\n");
                sprintf(buffer, "%s\n", __FILE__);
                break;
        }
        log_file << std::string(buffer);
    }
    log_file << std::flush;
    log_file.close();
}


template <typename T>
int Dsp_Board::apply_yaml_param(const YAML::Node& node, const std::string param_name, T &param, int cmd) {

    T tmp_par;
    const YAML::Node param_node = node[param_name];
    if ( param_node ) {
        tmp_par = param_node.as<T>();
        // in case yaml param is a mask OR with given param arg value
        param |= tmp_par;
        if ( setItem(cmd, &param, sizeof(param)) ) {
            DPRINTF("fail set yaml parameter %s\n", param_name.c_str());
            return -1;
        } else {
            DPRINTF("SET %s = %d 0x%04X\n", param_name.c_str(), param, param);
            return 0;
        }
    } else {
        //DPRINTF("<%s> Not found\n", param_name.c_str());
        return -2;
    }
}

///////////////////////////////////////////////////////////
//
///////////////////////////////////////////////////////////

void McBoard::apply_yaml_node(const YAML::Node& mc_node) {

    std::vector<int> pid(3);
    pid_gains_t      p_i_d;
    
    YAML::Node par_node = mc_node["pid"];

    if ( par_node ) {
        for ( YAML::const_iterator it=par_node.begin(); it!=par_node.end(); ++it ) {
            std::string key = it->first.as<std::string>();
            if ( key == "position" ) {
                p_i_d.gain_set = POSITION_GAINS;
            } else if ( key == "velocity" ) {
                p_i_d.gain_set = VELOCITY_GAINS;
            } else if ( key == "torque" ) {
                p_i_d.gain_set = TORQUE_GAINS;
            } else {
                continue;
            }
            pid = it->second.as<std::vector<int>>();
	    p_i_d.p = pid[0];
            p_i_d.i = pid[1];
            p_i_d.d = pid[2];
            setItem(SET_PID_GAINS,      &p_i_d.gain_set, sizeof(p_i_d));
        }
    }

    apply_yaml_param(mc_node, std::string("current_lim_mA"), _current_lim,   SET_CURRENT_LIMIT);

    apply_yaml_param(mc_node, std::string("max_torque_mNm"), _max_tor,       SET_MAX_TORQUE);

    getItem(GET_MOTOR_CONFIG, NULL, 0, REPLY_MOTOR_CONFIG, &_motor_config, sizeof(_motor_config));
    apply_yaml_param(mc_node, std::string("motor_config_mask"), _motor_config, SET_MOTOR_CONFIG);

    getItem(GET_MOTOR_CONFIG2, NULL, 0, REPLY_MOTOR_CONFIG2, &_motor_config2, sizeof(_motor_config2));
    apply_yaml_param(mc_node, std::string("motor_config2_mask"), _motor_config2, SET_MOTOR_CONFIG2);


    par_node = mc_node["impedance_control"];
    if ( par_node ) {
        //
        _torque_on_off = par_node.as<int>();
        //_torque_on_off = 1;
        setItem(SET_TORQUE_ON_OFF, &_torque_on_off, _torque_on_off);
    } 

    // check motor_config for impedance bit14 ?!?!
    if ( (_motor_config & (0x4000) >> 14) ^ (!!_torque_on_off) ) {
        DPRINTF("WARNING mismatch parameter for impedance control 0x%04X 0x%02X\n", _motor_config & (0x4000), _torque_on_off);
    }

    par_node = mc_node["filter_samples"];
    if ( par_node ) {
        DPRINTF("set filter samples\n");

        // 0 Torque sample
        _filter_setup[0] = 0;
        _filter_setup[1] = 50;
        _filter_setup[2] = 0;
        setItem(SET_FILTER_SAMPLES, &_filter_setup, 3);
        getItem(GET_FILTER_SAMPLES, &_filter_setup[0], 1, REPLY_FILTER_SAMPLES,   &_filter_setup, 3);
        //DPRINTF("%d %d %d\n", _filter_setup[0], _filter_setup[1], _filter_setup[2]);

        // 1 motor velocity sample
        _filter_setup[0] = 1;
        _filter_setup[1] = 50; //4
        _filter_setup[2] = 0;
        setItem(SET_FILTER_SAMPLES, &_filter_setup, 3);
        getItem(GET_FILTER_SAMPLES, &_filter_setup[0], 1, REPLY_FILTER_SAMPLES,   &_filter_setup, 3);
        //DPRINTF("%d %d %d\n", _filter_setup[0], _filter_setup[1], _filter_setup[2]);

        // 2 Link velocity sample
        _filter_setup[0] = 2;
        _filter_setup[1] = 10;
        _filter_setup[2] = 0;
        setItem(SET_FILTER_SAMPLES, &_filter_setup, 3);
        getItem(GET_FILTER_SAMPLES, &_filter_setup[0], 1, REPLY_FILTER_SAMPLES,   &_filter_setup, 3);
        //DPRINTF("%d %d %d\n", _filter_setup[0], _filter_setup[1], _filter_setup[2]);

        // 3 voltage sample
        _filter_setup[0] = 3;
        _filter_setup[1] = 1;
        _filter_setup[2] = 0;
        setItem(SET_FILTER_SAMPLES, &_filter_setup, 3);
        getItem(GET_FILTER_SAMPLES, &_filter_setup[0], 1, REPLY_FILTER_SAMPLES,   &_filter_setup, 3);
        //DPRINTF("%d %d %d\n", _filter_setup[0], _filter_setup[1], _filter_setup[2]);

        // 4 Current sample
        _filter_setup[0] = 4;
        _filter_setup[1] = 1;
        _filter_setup[2] = 0;
        setItem(SET_FILTER_SAMPLES, &_filter_setup, 3);
        getItem(GET_FILTER_SAMPLES, &_filter_setup[0], 1, REPLY_FILTER_SAMPLES,   &_filter_setup, 3);
        //DPRINTF("%d %d %d\n", _filter_setup[0], _filter_setup[1], _filter_setup[2]);

    }

}


/**
 * configure DSP board and get some parameter using TCP/IP
 * packets via Dsp_Board::setItem() and Dsp_Board::getItem()
 *
 * @param doc    YAML::Node
 */
void McBoard::configure(const YAML::Node &doc) {

    unsigned short tmp;
    const YAML::Node mc_board_node = doc["mc_board"];
    
    // look for board_<id> in config
    std::string board_conf_key = std::string("board_"+std::to_string(bId));
    const YAML::Node board_node = doc[board_conf_key];

    assert(mc_board_node);

    //
    // set board parameters
    // - first apply mc_board yaml-node parameters, at least set "bc_policy" and "bc_rate"   
    // - then, if present, apply board_<id> yaml-node parameters, overriding mc_board ones 
    //

    // yaml operator ">>" with uint8_t ?!@?#!@
    //(*mc_board_node)["bc_rate"] >> tmp;
    //bc_rate = tmp;
    bc_rate = (uint8_t)mc_board_node["bc_rate"].as<int16_t>();

    policy = ts_bc_data.raw_bc_data.mc_bc_data.get_policy();
    setItem(SET_BCAST_POLICY, &policy, sizeof(policy));
    extra_policy = ts_bc_data.raw_bc_data.mc_bc_data.get_extra_policy();
    setItem(SET_EXTRA_BCAST_POLICY, &extra_policy, sizeof(extra_policy));
    
    //
    // !!! DEPRECATED .... policy and extra policy are defined in broadcast_data.h
    //
    if ( mc_board_node["policy"] ) {
        DPRINTF("\n\tWARNING !! Found <mc_board:policy> parameter"
                "\n\tbroadcast policy is defined as 0x%04X in broadcast_data.h\n\n", policy);
    }
    if ( mc_board_node["extra_policy"] ) {
        DPRINTF("\n\tWARNING !! Found <mc_board:extra policy> parameter"
                "\n\tbroadcast extra_policy defined as 0x%04X in broadcast_data.h\n\n", extra_policy);

    }
    //apply_yaml_param(mc_board_node, std::string("policy"), policy, SET_BCAST_POLICY);
    //apply_yaml_param(mc_board_node, std::string("extra_policy"), extra_policy, SET_EXTRA_BCAST_POLICY);
    //assert(ts_bc_data.raw_bc_data.mc_bc_data.get_policy() == policy);
    //assert(ts_bc_data.raw_bc_data.mc_bc_data.get_extra_policy() == extra_policy);

    //
    _torque_on_off = 0;
    setItem(SET_TORQUE_ON_OFF, &_torque_on_off, sizeof(_torque_on_off));
    setItem(CLEAR_BOARD_FAULT, 0, 0);

    // given a yaml-node look for specific parametes like "pid" "current_lim_mA" "max_torque_mNm" ...
    apply_yaml_node(mc_board_node);
    if ( board_node ) {
        DPRINTF("Found %s use to set McBoard %d parameters \n", board_conf_key.c_str(), bId);
        apply_yaml_node(board_node);
    }

    //
    // read back 
    //

    getItem(GET_MIN_POSITION,   NULL, 0, REPLY_MIN_POSITION, &_min_pos, sizeof(_min_pos));
    getItem(GET_MAX_POSITION,   NULL, 0, REPLY_MAX_POSITION, &_max_pos, sizeof(_max_pos));

    getItem(GET_MIN_VELOCITY,   NULL, 0, REPLY_MIN_VELOCITY, &_min_vel, sizeof(_min_vel));
    getItem(GET_MAX_VELOCITY,   NULL, 0, REPLY_MAX_VELOCITY, &_max_vel, sizeof(_max_vel));

    getItem(GET_DESIRED_POSITION,   NULL, 0, REPLY_DESIRED_POSITION, &_des_pos, sizeof(_des_pos));
    getItem(GET_DESIRED_VELOCITY,   NULL, 0, REPLY_DESIRED_VELOCITY, &_des_vel, sizeof(_des_vel));
    getItem(GET_DESIRED_TORQUE,     NULL, 0, REPLY_DESIRED_TORQUE,   &_des_tor, sizeof(_des_tor));

    V_pid.gain_set = VELOCITY_GAINS;
    getItem(GET_PID_GAINS,      &V_pid.gain_set, 1, REPLY_PID_GAINS, &V_pid, sizeof(V_pid));
    P_pid.gain_set = POSITION_GAINS;
    getItem(GET_PID_GAINS,      &P_pid.gain_set, 1, REPLY_PID_GAINS, &P_pid, sizeof(P_pid));
    T_pid.gain_set = TORQUE_GAINS;
    getItem(GET_PID_GAINS,      &T_pid.gain_set, 1, REPLY_PID_GAINS, &T_pid, sizeof(T_pid));

    getItem(GET_MOTOR_CONFIG,   NULL, 0, REPLY_MOTOR_CONFIG,    &_motor_config,     sizeof(_motor_config));
    getItem(GET_MOTOR_CONFIG2,  NULL, 0, REPLY_MOTOR_CONFIG2,   &_motor_config2,    sizeof(_motor_config2));

    getItem(GET_TORQUE_FACTORS, NULL, 0, REPLY_TORQUE_FACTORS,  &_torque_factor,    sizeof(_torque_factor));
    getItem(GET_CURRENT_LIMIT,  NULL, 0, REPLY_CURRENT_LIMIT,   &_current_lim,      sizeof(_current_lim));
    getItem(GET_MAX_TORQUE,     NULL, 0, REPLY_MAX_TORQUE,      &_max_tor,          sizeof(_max_tor));
    getItem(GET_TORQUE_ON_OFF,  NULL, 0, REPLY_TORQUE_ON_OFF,   &_torque_on_off,    sizeof(_torque_on_off));

}

void McBoard::set_PID(int gain_set, int p, int i, int d) {

    pid_gains_t      p_i_d;

    switch ( gain_set ) {
        case POSITION_GAINS:
            p_i_d = P_pid;
            break;
        case TORQUE_GAINS:
            p_i_d = T_pid;
            break;
        case VELOCITY_GAINS:
            p_i_d = V_pid;
            break;
        default:
            assert(0);
    }

    p_i_d.p = p;
    p_i_d.i = i;
    p_i_d.d = d;

    setItem(SET_PID_GAINS,      &p_i_d.gain_set, sizeof(p_i_d));

    switch ( gain_set ) {
        case POSITION_GAINS:
            getItem(GET_PID_GAINS,      &P_pid.gain_set, 1, REPLY_PID_GAINS, &P_pid, sizeof(P_pid));
            DPRINTF("\tPID gains %d : %d %d %d\n",P_pid.gain_set, P_pid.p, P_pid.i, P_pid.d);
            break;
        case TORQUE_GAINS:
            getItem(GET_PID_GAINS,      &T_pid.gain_set, 1, REPLY_PID_GAINS, &T_pid, sizeof(T_pid));
            DPRINTF("\tPID gains %d : %d %d %d\n",T_pid.gain_set, T_pid.p, T_pid.i, T_pid.d);
            break;
        case VELOCITY_GAINS:
            getItem(GET_PID_GAINS,      &V_pid.gain_set, 1, REPLY_PID_GAINS, &V_pid, sizeof(V_pid));
            DPRINTF("\tPID gains %d : %d %d %d\n",V_pid.gain_set, V_pid.p, V_pid.i, V_pid.d);
            break;
        default:
            break;
    }

}

void McBoard::set_PID_increment(int gain_set, int p_incr, int i_incr, int d_incr) {

    pid_gains_t      p_i_d;

    switch ( gain_set ) {
        case POSITION_GAINS:
            p_i_d = P_pid;
            break;
        case TORQUE_GAINS:
            p_i_d = T_pid;
            break;
        case VELOCITY_GAINS:
            p_i_d = V_pid;
            break;
        default:
            assert(0);
    }

    p_i_d.p += p_incr;
    p_i_d.i += i_incr;
    p_i_d.d += d_incr;

    set_PID(p_i_d.gain_set, p_i_d.p, p_i_d.i, p_i_d.d);

}

void McBoard::get_PID(int gain_set, int &p, int &i, int &d) {

    pid_gains_t      p_i_d;

    switch ( gain_set ) {
        case POSITION_GAINS:
            p_i_d = P_pid;
            break;
        case TORQUE_GAINS:
            p_i_d = T_pid;
            break;
        case VELOCITY_GAINS:
            p_i_d = V_pid;
            break;
        default:
            assert(0);
    }

    p = p_i_d.p;
    i = p_i_d.i;
    d = p_i_d.d;

    return;
}

void McBoard::test_setting(void) {

    getItem(GET_DESIRED_POSITION,   NULL, 0, REPLY_DESIRED_POSITION, &_des_pos, sizeof(_des_pos));
    getItem(GET_DESIRED_VELOCITY,   NULL, 0, REPLY_DESIRED_VELOCITY, &_des_vel, sizeof(_des_vel));
    getItem(GET_DESIRED_TORQUE,     NULL, 0, REPLY_DESIRED_TORQUE,   &_des_tor, sizeof(_des_tor));
    DPRINTF("Des[%d] {pos:%d vel:%d tor:%d} \n",bId,_des_pos,_des_vel,_des_tor);
}


void McBoard::print_me(void) {

    Dsp_Board::print_me();
    DPRINTF("\tbc_policy 0x%04X\n",policy);
    DPRINTF("\tbc_extra_policy 0x%04X\n",extra_policy);
    DPRINTF("\tbc_freq %.1f ms\n", (float)bc_rate/2);
    DPRINTF("\t%s [%d:%d] mRAD\n","Position",_min_pos,_max_pos);
    DPRINTF("\t%s [%d:%d] mRAD/s\n","Velocity",_min_vel,_max_vel);
    DPRINTF("\tTorque %d Nm\n",_max_tor);
    DPRINTF("\tPID gains %d : %d %d %d\n",V_pid.gain_set, V_pid.p, V_pid.i, V_pid.d);
    DPRINTF("\tPID gains %d : %d %d %d\n",P_pid.gain_set, P_pid.p, P_pid.i, P_pid.d);
    DPRINTF("\tPID gains %d : %d %d %d\n",T_pid.gain_set, T_pid.p, T_pid.i, T_pid.d);
    DPRINTF("\ttorque factors %d %d\n",_torque_factor.multiplier, _torque_factor.offset);
    DPRINTF("\tcurrent limit  %d mA\n",_current_lim);
    DPRINTF("\tmotor config  0x%04X\n",_motor_config);
    DPRINTF("\tmotor config2 0x%04X\n",_motor_config2);
    DPRINTF("\ttorque on_off %d\n",_torque_on_off);
    DPRINTF("\tDes {pos:%d vel:%d tor:%d} \n",_des_pos,_des_vel,_des_tor);

}


///////////////////////////////////////////////////////////
//
///////////////////////////////////////////////////////////

void FtBoard::apply_yaml_node(const YAML::Node& ft_node) {

    bool calibrate_offset;

    try {
        calibrate_offset = ft_node["calibrate_offset"].as<bool>();
        if ( calibrate_offset ) {
            setItem(CALIBRATE_OFFSETS, NULL, 0);
            DPRINTF("FtSensor calibrate offsets\n");
        }
    } catch ( YAML::Exception &e ) {
        DPRINTF("%s\n", e.what());
    }

}

/**
 * configure DSP board and get some parameter using TCP/IP
 * packets via Dsp_Board::setItem() and Dsp_Board::getItem()
 *
 * @param doc    YAML::Node
 */
void FtBoard::configure(const YAML::Node &doc) {

    unsigned short tmp;
    const YAML::Node ft_board_node = doc["ft_board"];
   
    assert(ft_board_node);

    // yaml operator ">>" with uint8_t ?!@?#!@
    //(*ft_board_node)["bc_rate"] >> tmp;
    bc_rate = (uint8_t)ft_board_node["bc_rate"].as<int16_t>();

    policy = ts_bc_data.raw_bc_data.ft_bc_data.get_policy();
    setItem(SET_BCAST_POLICY, &policy, sizeof(policy));

    if ( ft_board_node["policy"] ) {
        DPRINTF("\n\tWARNING !! Found <ft_board:policy> parameter"
                "\n\tbroadcast policy is defined as 0x%04X in broadcast_data.h\n\n", policy);
    }

    // given a yaml-node look for specific parametes 
    apply_yaml_node(ft_board_node);

}

void FtBoard::print_me(void) {

    Dsp_Board::print_me();
    DPRINTF("\tbc_policy 0x%04X\n",policy);
    DPRINTF("\tbc_freq %.1f ms\n", (float)bc_rate/2);

}


