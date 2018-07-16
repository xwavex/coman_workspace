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

#ifndef __BOARDS_CTRL_WALK_H__
#define __BOARDS_CTRL_WALK_H__

#include <utils.h>
#include <thread_util.h>

#include <Boards_ctrl_ext.h>


/**
 * @class Boards_ctrl_walk
 * @defgroup Boards_controller Boards_controller
 * @ingroup RoboLLI
 * @brief Boards_ctrl_walk class
 */

class Boards_ctrl_walk : public Thread_hook, public Boards_ctrl_ext  {

private:
    uint64_t g_tStart;


public:
    Boards_ctrl_walk(const char * config);
    virtual void th_init(void *);
    virtual void th_loop(void *);
    int user_input(uint8_t &cmd);
    int user_loop_walk(void);
    int user_loop_test_joint(void);

};


#endif
