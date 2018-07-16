#include <Eigen/Dense>
using namespace Eigen;

#include <iostream>

#include <zmq.hpp>
//#include <zmq.h>
//#include <zmq_utils.h>

#ifndef ZMQ_DONTWAIT
    #define ZMQ_DONTWAIT ZMQ_NOBLOCK
#endif
#if ZMQ_VERSION_MAJOR == 2
    #define zmq_msg_send(msg,sock,opt) zmq_send (sock, msg, opt)
    #define zmq_msg_recv(msg,sock,opt) zmq_recv (sock, msg, opt)
    #define zmq_ctx_destroy(context) zmq_term(context)
    #define ZMQ_POLL_MSEC 1000 // zmq_poll is usec
    #define ZMQ_SNDHWM ZMQ_HWM
    #define ZMQ_RCVHWM ZMQ_HWM
#elif ZMQ_VERSION_MAJOR == 3
    #define ZMQ_POLL_MSEC 1 // zmq_poll is msec
#endif

#include <Boards_ctrl_wbmc.h>
#include <wbmc_func.h>


float min_effort(int i,
                 Matrix<float,29,1>& q,
                 Matrix<float,29,1>& sinq,
                 Matrix<float,29,1>& cosq,
                 float epsq, float tau_GC, float effort, float k);


int main(int argc, char *argv[])
{

    static Matrix<float,29,1>  q, sinq, cosq, tau_GC;
    static float epsq, effort, k;

    float result;
    static uint64_t tPrev;

    min_effort_args_t min_effort_args;
    min_effort_res_t  min_effort_res;

    int i = atoi(argv[1]);

    zmq::context_t zmq_global_ctx(1);

    //  Socket to send result to
    zmq::socket_t sender(zmq_global_ctx, ZMQ_PUSH);
    //sender.connect("ipc:///tmp/9696");
    sender.connect("tcp://wheezy-i386-test.local:9696");

    //  Socket to receive input from
    zmq::socket_t zsub (zmq_global_ctx, ZMQ_SUB);
    //zsub.connect("ipc:///tmp/6969");
    zsub.connect("tcp://wheezy-i386-test.local:6969");
    zsub.setsockopt(ZMQ_SUBSCRIBE, "", 0);

    min_effort_res.i = i;

    while (1) {

        zmq::message_t msg_id;
        zmq::message_t msg;

        zsub.recv(&msg_id);
        zsub.recv(&msg, ZMQ_RCVMORE);

        //  Simple progress indicator for the viewer
        std::cout << i <<"." << std::flush;

        std::string id(static_cast<char*>(msg_id.data()), msg_id.size());
        if (id.compare("END") == 0) {
            break;
        }
        //std::cout << id << std::endl;
        memcpy((void*)&min_effort_args,(void*)msg.data(), msg.size());

        q = Map<Matrix<float,29,1>>(min_effort_args.q);
        sinq = Map<Matrix<float,29,1>>(min_effort_args.sinq);
        cosq = Map<Matrix<float,29,1>>(min_effort_args.cosq);
        tau_GC = Map<Matrix<float,29,1>>(min_effort_args.tau_GC);
        effort = min_effort_args.effort;
        k = min_effort_args.k;
        epsq =  min_effort_args.epsq;

        tPrev = get_time_ns();
        //  Do the work
        min_effort_res.min_effort = min_effort(i,q,sinq,cosq,epsq,tau_GC(i),effort,k);
        min_effort_res.dt = get_time_ns() - tPrev;
        
        //  Send results
        msg.rebuild(sizeof(min_effort_res_t));
        memcpy((void*)msg.data(), (void*)&min_effort_res, sizeof(min_effort_res_t));
        sender.send(msg);

        //  Simple progress indicator for the viewer
        //std::cout << i <<"." << std::flush;

    }

    //  Finished
    return 0;

}
