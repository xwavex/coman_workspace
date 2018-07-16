/*
	DSP_board.h

	Copyright (C) 2012 Italian Institute of Technology

	Developer:
        Alessio Margan (2012-, alessio.margan@iit.it)

*/

#ifndef __DSP_BOARDS_H__
#define __DSP_BOARDS_H__

#include <netinet/in.h>
#include <arpa/inet.h>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/count.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/min.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/circular_buffer.hpp>

#include "yaml-cpp/yaml.h"

#include <definitions.h>

//------------------------------------------------

//------------------------------------------------
using namespace boost::accumulators;

/**
 * @class Dsp_Board
 * @defgroup Dsp_Board Dsp board
 * @ingroup RoboLLI
 * @brief Dsp_Board base class to interface with DSP boards,
 *        each single instance handle TCP/IP communication with
 *        the relative board and process its broadcast data
 *        dispatched by Boards_ctrl class
 */

class Dsp_Board {

public:
    Dsp_Board(uint8_t  *);
    virtual ~Dsp_Board();

    // thread safe, mutex on sock_fd
    int setItem(int reqCmd, void *src, int srcBytes);
    int getItem(int reqCmd, void *src, int nSrcBytes, int resCmd, void *dst, int nDstBytes);

    virtual void configure(const YAML::Node&) = 0;
    virtual void on_bc_data(uint8_t *);
    virtual void get_bc_data(ts_bc_data_t &);
    virtual void print_me(void);

    void start_stop_bc(uint8_t);

    uint8_t     stopped;
    uint8_t     bId;
    uint8_t     bType;

protected:

    template <typename T>
    int apply_yaml_param(const YAML::Node& node, const std::string param_name, T &param, int cmd);
    virtual void apply_yaml_node(const YAML::Node & ) = 0;
    virtual uint32_t get_bc_data_size() { return 0; }

    char        ip_addr[16];
    //
    int         sock_fd;
    sockaddr_in sock_addr;
    //
    uint16_t    policy;
    uint16_t    extra_policy;
    uint8_t     bc_rate;
    //
    ts_bc_data_t    ts_bc_data;

private:

    void dump_log(void);
    void measure_bc_freq(void);
    void print_stat(void);

    //
    uint64_t _bc_tStart;
    uint64_t _rx_bc_prec;
    accumulator_set<uint64_t,
        features<
            tag::count
            ,tag::mean
            ,tag::min
            ,tag::max
            //,tag::variance(lazy)
            //,tag::error_of<tag::mean>
        >
    > bc_freq;

    pthread_mutex_t dsp_data_mutex;
    pthread_mutex_t dsp_sock_mutex;

    boost::circular_buffer<ts_bc_data_t> dsp_log;


};


/**
 * @class McBoard
 * @defgroup McBoard Motor controller board
 * @ingroup Dsp_Board
 * @brief McBoard class derived from Dsp_Board, implement
 *        derived virtual method to carry out motor controller
 *        protocol
 */
class McBoard : public Dsp_Board{

public:
    McBoard(uint8_t *_):Dsp_Board(_) { }

    void test_setting(void);

    void set_PID(int gain_set, int, int, int);
    void set_PID_increment(int gain_set, int, int, int);
    void get_PID(int gain_set, int &, int &, int &);

protected:
    virtual void apply_yaml_node(const YAML::Node& );
    virtual void configure(const YAML::Node&);
    virtual void print_me(void);
    virtual uint32_t get_bc_data_size() { return sizeof(mc_bc_data_t); }


public:
    
    // dsp parameter read during configure() method
    // do not write on this variables ....
    int         _min_pos, _max_pos;
    short int   _min_vel, _max_vel;
    short int   _max_tor;
    short int   _des_pos, _des_vel, _des_tor;
    short int   _current_lim;
    short int   _motor_config, _motor_config2;
    pid_gains_t     V_pid, P_pid, T_pid;
    torque_factor_t _torque_factor;
    uint8_t     _torque_on_off;
    unsigned char   _filter_setup[3];

};


/**
 * @class FtBoard
 * @defgroup FtBoard Force torque sensor board
 * @ingroup Dsp_Board
 * @ingroup Dsp_Board
 * @brief FtBoard class derived from Dsp_Board, implement
 *        derived virtual method to carry out force torque
 *        sensor protocol
 */
class FtBoard : public Dsp_Board {
public:
    FtBoard(uint8_t *_):Dsp_Board(_) { }

protected:
    virtual void apply_yaml_node(const YAML::Node& );
    virtual void configure(const YAML::Node&);
    virtual void print_me(void);
    virtual uint32_t get_bc_data_size() { return sizeof(ft_bc_data_t); }


};

#endif
