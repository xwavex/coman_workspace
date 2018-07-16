#include <fcntl.h>
#include <unistd.h>

#include <thread_util.h>
#include <utils.h>

#include <list>

#include <zmq_publisher.h>


Zmq_pub_thread::Zmq_pub_thread(std::string _pipe_name, Publisher * _publisher):
    pipe_name(_pipe_name)
{
   
    std::string tmp_str = std::string("zmq_pub_") + pipe_name;
    name = strdup(tmp_str.c_str());
    period.period = {0,1};

    schedpolicy = SCHED_OTHER;
    priority = sched_get_priority_max(schedpolicy);
    stacksize = PTHREAD_STACK_MIN;

    publisher = _publisher;
}

void Zmq_pub_thread::th_init(void *) {

    // wait for file creation
    //sleep(1);

    std::string pipe = pipe_prefix + pipe_name;

    printf ("Opening xddp_socket %s\n", pipe.c_str());
    xddp_sock = open(pipe.c_str(), O_RDONLY | O_NONBLOCK);

    if (xddp_sock < 0) {
        printf ("%s %s : %s\n", __FILE__, __FUNCTION__, strerror (errno));
        exit(1);
    }

}

void Zmq_pub_thread::th_loop(void *) {

    int nbytes = 0;
    int expected_bytes = publisher->data_size();

    // ////////////////////////////////////////////////////////////////////////
    // BUSY WAIT .... looping until read some data 
    do {
        // NON-BLOCKING : read binary data
        nbytes = read(xddp_sock, (void*)buffer, expected_bytes);
    } while(nbytes <= 0 && _run_loop);

    if (nbytes != expected_bytes) {
        printf("zmq rx %d expected %d\n", nbytes, expected_bytes);
    }

    publisher->publish(buffer);

}

Zmq_pub_thread::~Zmq_pub_thread() {

    std::cout << "~" << typeid(this).name() << std::endl;

    delete publisher;
    free((void*)name);
    close(xddp_sock);

}


