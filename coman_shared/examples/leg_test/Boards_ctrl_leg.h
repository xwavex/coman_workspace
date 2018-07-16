/*
   Boards_ctrl_leg.h

   Copyright (C) 2012 Italian Institute of Technology

   Developer:
       Alessio Margan (2013-, alessio.margan@iit.it)
   
*/

/**
 *
 * @author Alessio Margan (2013-, alessio.margan@iit.it)
*/

#ifndef __BOARDS_CTRL_LEG_H__
#define __BOARDS_CTRL_LEg_H__

#include <utils.h>
#include <thread_util.h>

#include <Boards_ctrl_ext.h>


/**
 * @class Boards_ctrl_leg
 * @defgroup Boards_controller Boards_controller
 * @ingroup RoboLLI
 * @brief Boards_ctrl_leg class
 */

class Boards_ctrl_leg : public Thread_hook, public Boards_ctrl_ext  {

public:
    Boards_ctrl_leg(const char * config);

    virtual void th_init(void *);
    virtual void th_loop(void *);

    virtual void move(void);
    virtual int user_input(uint8_t &);

    void impedance_ctrl(void);
    void set_abs_offset(int dir);

    uint64_t g_tStart;

private:
    
    uint8_t _trj_flag;

};


#endif
