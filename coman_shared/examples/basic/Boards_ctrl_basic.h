/*
   Boards_ctrl_basic.h

   Copyright (C) 2012 Italian Institute of Technology

   Developer:
       Alessio Margan (2013-, alessio.margan@iit.it)
   
*/

/**
 *
 * @author Alessio Margan (2013-, alessio.margan@iit.it)
*/

#ifndef __BOARDS_CTRL_BASIC_H__
#define __BOARDS_CTRL_BASIC_H__

#include <utils.h>
#include <thread_util.h>

#include <Boards_ctrl_ext.h>


/**
 * @class Boards_ctrl_basic
 * @defgroup Boards_controller Boards_controller
 * @ingroup RoboLLI
 * @brief Boards_ctrl_basic class
 */

class Boards_ctrl_basic : public Thread_hook, public Boards_ctrl_ext  {

private:
    uint64_t g_tStart;
    uint8_t trj_flag;

    group_ref_t pos_group;
    group_ref_comp_t pos_vel_group;

    double control_old, vel_old;

    Write_XDDP_pipe * xddp_test;

public:
    Boards_ctrl_basic(const char * config);
    virtual ~Boards_ctrl_basic();

    virtual void th_init(void *);
    virtual void th_loop(void *);

    int user_input(uint8_t &cmd);
    int user_loop(void);

};


#endif
