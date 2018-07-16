/*
   Boards_ctrl_ext.h

   Copyright (C) 2012 Italian Institute of Technology

   Developer:
       Alessio Margan (2013-, alessio.margan@iit.it)

*/

/**
 * @defgroup RoboLLI RoboLLI
 *
 * @author Alessio Margan (2013-, alessio.margan@iit.it)
*/

#ifndef __BOARDS_CTRL_EXT_H__
#define __BOARDS_CTRL_EXT_H__

#include <Boards_iface.h>
#ifdef __XENO__
    #include <rt_ipc.h>
#endif



class XDDP_pipe {

public:
    XDDP_pipe(const std::string pipe_name, int pool_size):
        pipe_name(pipe_name),
        pool_size(pool_size)
    {
    #ifdef __XENO__
        fd = xddp_bind(pipe_name.c_str(), pool_size);
    #else
        std::string pipe = pipe_prefix + pipe_name;
        mkfifo(pipe.c_str(), S_IRWXU|S_IRWXG);
        fd = open(pipe.c_str(), O_RDWR | O_NONBLOCK);
    #endif
        assert(fd);
    }

    virtual ~XDDP_pipe()
    {
        close(fd);
    #ifndef __XENO__
        std::string pipe = pipe_prefix + pipe_name;
        unlink(pipe.c_str());
    #endif

    }

protected:
    int fd;
    int pool_size;
    std::string pipe_name;
};


class Write_XDDP_pipe : public XDDP_pipe {
public:
    Write_XDDP_pipe(std::string pipe_name, int pool_size):
        XDDP_pipe(pipe_name, pool_size) { }

    int write(void *buffer, int nbytes)
    {
        return ::write(fd, buffer, nbytes);
    }
};

class Read_XDDP_pipe : public XDDP_pipe {
public:
    Read_XDDP_pipe(std::string pipe_name, int pool_size):
        XDDP_pipe(pipe_name, pool_size) { }

    int read(void *buffer, ssize_t buff_size)
    {
    /////////////////////////////////////////////////////////
    // NON-BLOCKING, read buff_size byte from pipe or cross domain socket
    #if __XENO__
        return recvfrom(fd, buffer, buff_size, MSG_DONTWAIT, NULL, 0);
    #else
        // NON-BLOCKING
        return ::read(fd, buffer, buff_size);
    #endif
    }
};


/**
 * @class Boards_ctrl_ext
 * @defgroup Boards_controller_ext Boards_controller_ext
 * @ingroup RoboLLI
 * @brief Boards_ctrl_ext class
 */

class Boards_ctrl_ext : public Boards_ctrl {

public:
    Boards_ctrl_ext(const char * config);
    virtual ~Boards_ctrl_ext();

    virtual void init(void);

    virtual void homing(void);
    virtual void homing(const std::vector<float> &pos, const std::vector<float> &vel);

    virtual void sense(void);
    virtual void move(int move_mask=MV_POS|MV_VEL|MV_TOR);

    virtual int check_mcs_faults();
    virtual int user_input(void *buffer, ssize_t buff_size);

protected:

    Read_XDDP_pipe  * console;
    Write_XDDP_pipe * xddp_bc_data;
    Write_XDDP_pipe * xddp_user_data;

    int     _home[MAX_DSP_BOARDS];
    int     _pos[MAX_DSP_BOARDS];
    short   _vel[MAX_DSP_BOARDS];
    short   _tor[MAX_DSP_BOARDS];
    int     _stiff[MAX_DSP_BOARDS];
    int     _damp[MAX_DSP_BOARDS];
    short   _mVolt[MAX_DSP_BOARDS];

    ts_bc_data_t   _ts_bc_data[MAX_DSP_BOARDS];
    pid_gains_t    _user_pid_gains[MAX_DSP_BOARDS];
};


#endif
