#ifndef __WBMC_PUBLISHER_H__
#define __WBMC_PUBLISHER_H__

#include <zmq_publisher.h>
#include <Boards_ctrl_wbmc.h>

#define SUBSCRIBERS_EXPECTED  10


class WBMC_Publisher: public Publisher {

private:
    min_effort_args_t data;

public:

    WBMC_Publisher(std::map<int, std::string> uri_list):
        Publisher(uri_list) {

        }

    virtual ~WBMC_Publisher() {

        std::string id("END");
        _msg_id.rebuild(id.length());
        memcpy((void*)_msg_id.data(), (void*)id.c_str() , id.length());

        _msg.rebuild(sizeof(data));
        memset((void*)_msg.data(), 0, sizeof(min_effort_args_t));
        publish_msg();
        printf ("\n~WBMC_Publisher\n");
    }

    virtual int data_size() { return sizeof(min_effort_args_t); }

    virtual void publish(uint8_t * buffer) {

        std::string id("parallel_arg");
        _msg_id.rebuild(id.length());
        memcpy((void*)_msg_id.data(), (void*)id.c_str() , id.length());

        _msg.rebuild(sizeof(data));
        memcpy((void*)_msg.data(), (void*)buffer , sizeof(min_effort_args_t));
        publish_msg();
    }

};




class Zmq_pub_pull_thread : public Zmq_pub_thread {

private:

    int                 xddp_sock_wr;
    min_effort_res_t    min_effort_tmp;
    float               min_effort_res[29];

    zmq::message_t  calc_res;
    zmq::socket_t * calc_recv;

    std::string pipe;

    accumulator_set<uint64_t,
    features<
        tag::count
        ,tag::mean
        ,tag::min
        ,tag::max
        ,tag::variance(lazy)
        ,tag::error_of<tag::mean>
        >
    > tPull, tPub, dtCalc[29];


public:

    Zmq_pub_pull_thread(std::string pipe_name, Publisher * publisher):
        Zmq_pub_thread(pipe_name, publisher) {

        pipe = pipe_prefix + "parallel_calc_res";

        printf ("Opening xddp_socket %s\n", pipe.c_str());
        xddp_sock_wr = open(pipe.c_str(), O_WRONLY | O_NONBLOCK);

        if (xddp_sock_wr < 0) {
            printf ("error in _init: %s\n", strerror (errno));
            exit(1);
        }

        calc_recv = new zmq::socket_t(zmq_global_ctx, ZMQ_PULL);
        //calc_recv->bind("ipc:///tmp/9696");
        calc_recv->bind("tcp://*:9696");

    }

    virtual ~Zmq_pub_pull_thread() {

        delete calc_recv;
        close(xddp_sock_wr);
        printf ("\n~Zmq_pub_pull_thread\n");
        std::cout << "Mean tPub ns  : " << (uint64_t)mean(tPub) << std::endl;
        std::cout << "Mean tPull ns : " << (uint64_t)mean(tPull) << std::endl;
        std::cout << "Count tPull   : " << (uint64_t)count(tPull) << std::endl;
        for (int i=6; i<29; i++) {
            std::cout << "\tMean dtCalc ns : " << (uint64_t)mean(dtCalc[i]) << std::endl;

        }
    }

    virtual void th_loop(void *) {

        static uint64_t tPrev; 

        tPrev = get_time_ns();
        // read from xddp/pipe parallel_calc_arg and publish
        Zmq_pub_thread::th_loop(0);
        tPub(get_time_ns() - tPrev);

        // pull calc results from (push)workers .....
        // we use 23 because first 6 are zero
        int count = 23;
        memset((void*)min_effort_res, 0, sizeof(min_effort_res));

        printf ("\n=============\n");
        tPrev = get_time_ns();
        while (count--) {
            calc_recv->recv(&calc_res);
            memcpy((void*)&min_effort_tmp, (void*)calc_res.data(), sizeof(min_effort_res_t));
            min_effort_res[min_effort_tmp.i] = min_effort_tmp.min_effort;
            dtCalc[min_effort_tmp.i](min_effort_tmp.dt);
            printf ("%d ", min_effort_tmp.i);
        }
        printf ("\n");

        tPull(get_time_ns() - tPrev);

        // write to xddp/pipe parallel_calc_res
        int nbytes = write(xddp_sock_wr, (void*)min_effort_res, sizeof(min_effort_res));
        printf("%d > parallel_cal_res\n", nbytes);

    }

};

#endif
