#include <thread_util.h>
#include <Boards_ctrl_leg.h>

Boards_ctrl_leg * boards_ctrl = new Boards_ctrl_leg("config.yaml");

///////////////////////////////////////////////////////////////////////////////
// 
///////////////////////////////////////////////////////////////////////////////

void * boards_ctrl_thread(void * arg) {

#if __XENO__
    return rt_periodic_thread(boards_ctrl);
#else
    return nrt_thread(boards_ctrl);
#endif
}


