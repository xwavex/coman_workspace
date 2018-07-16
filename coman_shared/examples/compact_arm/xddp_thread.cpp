#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <pthread.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>

//#include <string>
#include <iostream>

#include <unistd.h>
#include <termios.h>

// turn off canonical mode (and echo mode to suppress echoing).
char getch() {
        char buf = 0;
        struct termios old = {0};
        if (tcgetattr(0, &old) < 0)
                perror("tcsetattr()");
        old.c_lflag &= ~ICANON;
        old.c_lflag &= ~ECHO;
        old.c_cc[VMIN] = 1;
        old.c_cc[VTIME] = 0;
        if (tcsetattr(0, TCSANOW, &old) < 0)
                perror("tcsetattr ICANON");
        if (read(0, &buf, 1) < 0)
                perror ("read()");
        old.c_lflag |= ICANON;
        old.c_lflag |= ECHO;
        if (tcsetattr(0, TCSADRAIN, &old) < 0)
                perror ("tcsetattr ~ICANON");
        return (buf);
}

void * xddp_nrt_thread(void *)
{
    int xddp_sock, nbytes;
    char c;

#if RT_ENV
    xddp_sock = open("/proc/xenomai/registry/rtipc/xddp/boards_ctrl", O_WRONLY);
#else
    xddp_sock = open("/tmp/boards_ctrl", O_WRONLY);
#endif
    if (xddp_sock < 0 ) {
        printf ("error : %s\n", strerror (errno));
        exit(1);
    }

    while (1) {

        //std::getline(std::cin, line);
        //std::cout << line << std::endl;
        //write(xddp_sock, (void*)line.c_str(), line.length());

        c = getch();
        std::cout << ">>" << c << std::endl;
        nbytes = write(xddp_sock, (void*)&c, sizeof(c));
        sleep(0.2);
    }

    return 0;
}


int main(void) {

    xddp_nrt_thread(0);
    return 0;
}
