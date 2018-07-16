#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/time.h>
#include <errno.h>
#include <math.h>
#include <assert.h>
#include <pthread.h>
#include <bits/local_lim.h>

#include <boost/circular_buffer.hpp>

#include <Boards_iface.h>
#include <imu_3DM-GX3-25.h>

#include <utils.h>
#include <thread_util.h>


#define DEG2mRAD(X) (X*M_PI*1e5)/180
#define DEG2RAD(X)  (X*M_PI)/180

#ifdef RT_ENV
    #include <rt_ipc.h>
    #include <rtdk.h>
    #define DPRINTF rt_printf
#else
    #include <fcntl.h>
    #include <sys/types.h>
    #include <sys/stat.h>
    #define DPRINTF printf
#endif


#if DEMO_TYPE==1
// below code test joints only
extern void test_joint(std::vector<float> homingPos, int size, double freq, int r_pos[]);
extern void JointTestState(char cmd);
#elif DEMO_TYPE==2
    extern void RTControl(double RTtime, float FTSensor[],std::vector<float> homePos, int size,int *pos);
    extern void InitializeWalkState();
    extern void SetInitialFlag();
    extern void KeyBoardControl(char cmd);
#endif

extern int loop;
///////////////////////////////////////////////////////////////////////////////
// Globals
///////////////////////////////////////////////////////////////////////////////

int fd_info, fd_data;
unsigned long long  g_tStart = 0;
boost::circular_buffer<log_ctrl_t> log_buff(LOG_SIZE);

Boards_ctrl * boards_ctrl = new Boards_ctrl("config.yaml");

//
nrt_imu_3DM_GX3_25 * imu;

#ifdef EPFL
// home position in degree for EPFL robot
std::vector<float> homingPos = {
    // lower body #15
    0,  -1,  0,  0,  0,  0,  0,-1.5, 0, 0,  0, 0,-1.5,  0,  0,
//  1,  2,  3,  4,  5,  6,  7,  8,  9, 10,  11, 12, 13, 14, 15
    // upper body #10 right arm to left arm, last 2 are right and left neck
    0, 90,  0,  -90, 0, -90, 0, -90,  0,  0};
// 16,17, 18,  19, 20, 21, 22, 23,   24, 25
#else
// home position in degree for old COMAN
std::vector<float> homingPos = {
    // lower body #15
    0,  0,  0,  0,  -1,-3.5, 0,-1.5, 0, -1.75, -3.5, 0,-1.5, 0,  0,
//  1,  2,  3,  4,  5,  6,   7,  8,  9, 10,   11, 12, 13, 14, 15
    // upper body #10
    -28, 70,  0, 90,  -28, 70,  0, 90,  0,  0};
// 16, 17, 18, 19, 20, 21, 22, 23, 24, 25
#endif

std::vector<int> r_leg = {  4,  6,  7,  8,  9, 10};
std::vector<int> l_leg = {  5, 11, 12, 13, 14, 15};
std::vector<int> waist = {  1,  2, 3};
std::vector<int> r_arm = { 16, 17, 18 ,19};
std::vector<int> l_arm = { 20, 21, 22, 23};
std::vector<int> neck  = { 24, 25};


///////////////////////////////////////////////////////////////////////////////
//
///////////////////////////////////////////////////////////////////////////////



void dump_master_log(void)
{
    char logfile[64];
    sprintf(logfile,"/tmp/master_log.txt");
    FILE * fp = fopen(logfile,"w");

    for (boost::circular_buffer<log_ctrl_t>::iterator it=log_buff.begin(); it!=log_buff.end(); it++) {
        fprintf(fp,"%llu\t", (*it).ts);
        fprintf(fp,"\n");
    }
    fclose(fp);
}


///////////////////////////////////////////////////////////////////////////////
//
///////////////////////////////////////////////////////////////////////////////


static void start_control_body(std::vector<int> body) {

    DPRINTF("start control ");
    for (int j=0; j<body.size(); j++) {
        DPRINTF(" %d", body[j]);
        boards_ctrl->start_stop_single_control((uint8_t)body[j],true);
    }
    DPRINTF("\n");

}

static void home_body()
{
    int r_pos[MAX_DSP_BOARDS];
    short r_vel[MAX_DSP_BOARDS];

    for (int i=0; i<homingPos.size(); i++) {
        r_pos[i] = DEG2mRAD(homingPos[i]);
    }
    for (int i=0; i<homingPos.size(); i++) {
        r_vel[i] = DEG2RAD(30)*1000;
    }
    boards_ctrl->set_velocity(r_vel, sizeof(r_vel));
    boards_ctrl->set_position(r_pos, sizeof(r_pos));
}


///////////////////////////////////////////////////////////////////////////////

void _init(void * _)
{
    int r_pos[MAX_DSP_BOARDS];
    short r_vel[MAX_DSP_BOARDS];
    struct timespec ts = {1,0};
    int numActive = 0;

#ifdef RT_ENV
    fd_info = xddp_bind("boards_ctrl");
    fd_data = xddp_bind("boards_bc_data");
#else
    const char * pipes_name[] = { "/tmp/boards_ctrl", "/tmp/boards_bc_data"};
    mkfifo(pipes_name[0], S_IRWXU);
    fd_info = open(pipes_name[0], O_RDWR | O_NONBLOCK);
    mkfifo(pipes_name[1], S_IRWXU);
    fd_data = open(pipes_name[1], O_RDWR | O_NONBLOCK);
#endif

    boards_ctrl->init();
    numActive = boards_ctrl->scan4active();
    DPRINTF("Found %d boards\n", numActive);

    if (numActive==0) {
        exit(1);
    }
    //imu = new nrt_imu_3DM_GX3_25("/dev/ttyACM0", 115200, CMD_ACCEL_ANGRATE_MAG_ORIENT);

    // use config.yaml settings
    boards_ctrl->configure_boards();

    home_body();
#if DEMO_TYPE==2
    InitializeWalkState();
    SetInitialFlag();
#endif

    // TO TEST .... NOT WORKING
    //boards_ctrl->set_position_velocity(r_pos, r_vel, MAX_DSP_BOARDS);
    boards_ctrl->test();
    // WAIT to let dsp thinking .... LEAVE HERE
    //clock_nanosleep(CLOCK_REALTIME, 0, &ts, NULL);
    sleep(1);

#if 1
    boards_ctrl->start_stop_control(true);
#else

    DPRINTF("Start control r_leg\n");
    start_control_body(r_leg);

    DPRINTF("Start control l_leg\n");
    start_control_body(l_leg);

    DPRINTF("Start control waist\n");
    start_control_body(waist);

    DPRINTF("Start control r_arm\n");
    start_control_body(r_arm);

    DPRINTF("Start control l_arm\n");
    start_control_body(l_arm);

    DPRINTF("Start control neck\n");
    start_control_body(neck);

#endif

    boards_ctrl->start_stop_bc_boards(true);

    //ts.tv_sec = 3.0;
    //clock_nanosleep(CLOCK_REALTIME, 0, &ts, NULL);
    sleep(3);

    g_tStart = get_time_ns();
}

#if DEMO_TYPE==2
void  _loop_walking(void * _)
{
    static int r_pos[MAX_DSP_BOARDS];
    static short r_vel[MAX_DSP_BOARDS];
    static bc_data_t   bc_data[MAX_DSP_BOARDS];
    log_ctrl_t  log_elem;
    int nbytes;
    char cmd;// keyboard command
    static float FTSensor[12];
    // imu data
    static float acc[3], angRate[3], mag[3], orientMat[9];
    //static char print_buff[1024];
    /////////////////////////////////////////////////////////
    // Sensor
    boards_ctrl->get_bc_data(bc_data);
    //bc_data[25].ft_bc_data.sprint(print_buff,1024);
    //DPRINTF("%s",print_buff);
#ifdef EPFL
    //EPFL robot
    FTSensor[0]  = 0.001*float(bc_data[25].ft_bc_data.fx);
    FTSensor[1]  = 0.001*float(bc_data[25].ft_bc_data.fy);
    FTSensor[2]  = 0.001*float(bc_data[25].ft_bc_data.fz);
    FTSensor[3]  = 0.001*float(bc_data[25].ft_bc_data.tx);
    FTSensor[4]  = 0.001*float(bc_data[25].ft_bc_data.ty);
    FTSensor[5]  = 0.001*float(bc_data[25].ft_bc_data.tz);
    FTSensor[6]  = 0.001*float(bc_data[26].ft_bc_data.fx);
    FTSensor[7]  = 0.001*float(bc_data[26].ft_bc_data.fy);
    FTSensor[8]  = 0.001*float(bc_data[26].ft_bc_data.fz);
    FTSensor[9]  = 0.001*float(bc_data[26].ft_bc_data.tx);
    FTSensor[10] = 0.001*float(bc_data[26].ft_bc_data.ty);
    FTSensor[11] = 0.001*float(bc_data[26].ft_bc_data.tz);
#else
    // below for the old COMAN
    FTSensor[0] = 0.001*float(bc_data[25].ft_bc_data.fx);
    FTSensor[1] = -0.001*float(bc_data[25].ft_bc_data.fy);
    FTSensor[2] = -0.001*float(bc_data[25].ft_bc_data.fz);
    FTSensor[3] = 0.001*float(bc_data[25].ft_bc_data.tx);
    FTSensor[4] = -0.001*float(bc_data[25].ft_bc_data.ty);
    FTSensor[5] = -0.001*float(bc_data[25].ft_bc_data.tz);
    FTSensor[6] = 0.001*float(bc_data[26].ft_bc_data.fx);
    FTSensor[7] = -0.001*float(bc_data[26].ft_bc_data.fy);
    FTSensor[8] = -0.001*float(bc_data[26].ft_bc_data.fz);
    FTSensor[9] = 0.001*float(bc_data[26].ft_bc_data.tx);
    FTSensor[10] = -0.001*float(bc_data[26].ft_bc_data.ty);
    FTSensor[11] = -0.001*float(bc_data[26].ft_bc_data.tz);
#endif

    //DPRINTF("%.2f Left \t %.2f \t Fztot %.2f\n", FTSensor[2],FTSensor[8], FTSensor[8]+FTSensor[2]);
    //DPRINTF("Right%d\t%d\t%d\t%d\t%d\t%d\n", bc_data[25].ft_bc_data.fx,bc_data[25].ft_bc_data.fy, bc_data[25].ft_bc_data.fz,bc_data[25].ft_bc_data.tx,bc_data[25].ft_bc_data.ty,bc_data[25].ft_bc_data.tz);
    /////////////////////////////////////////////////////////
    // write to pipe or cross domain socket
    write(fd_data, (void*)&bc_data, sizeof(bc_data));
    /////////////////////////////////////////////////////////
    // NON-BLOCKING read char from pipe or cross domain socket
#if RT_ENV
    nbytes = recvfrom(fd_info, (void*)&cmd, sizeof(cmd), MSG_DONTWAIT, NULL, 0);
#else
    // NON-BLOCKING
    nbytes = read(fd_info, (void*)&cmd, sizeof(cmd));
#endif
    if (nbytes > 0) {
        KeyBoardControl(cmd);
    }
    double RTtime = ( (get_time_ns() - g_tStart)/1e9 );
    //////////////// walking pattern ////////////////////////////
    RTControl(RTtime,FTSensor,homingPos, homingPos.size(), r_pos);
    // send position and velocity reference
    boards_ctrl->set_velocity(r_vel, sizeof(r_vel));
    boards_ctrl->set_position(r_pos, sizeof(r_pos));
    /////////////////////////////////////////////////////////
    // log
    log_elem.ts = get_time_ns() - g_tStart;
    for (int i=0; i<MAX_DSP_BOARDS; i++) {
        log_elem.pos[i] = r_pos[i];
    }
    log_buff.push_back(log_elem);
}
#endif

#if DEMO_TYPE==1
void  _loop_joint(void * _)
{
    int r_pos[MAX_DSP_BOARDS];
    short r_vel[MAX_DSP_BOARDS];
    bc_data_t   bc_data[MAX_DSP_BOARDS];
    log_ctrl_t  log_elem;
    int nbytes;
    char cmd;// keyboard command
    // write to pipe or cross domain socket
    write(fd_data, (void*)&bc_data, sizeof(bc_data));
    /////////////////////////////////////////////////////////
    // NON-BLOCKING read char from pipe or cross domain socket
#if RT_ENV
    nbytes = recvfrom(fd_info, (void*)&cmd, sizeof(cmd), MSG_DONTWAIT, NULL, 0);
#else
    // NON-BLOCKING
    nbytes = read(fd_info, (void*)&cmd, sizeof(cmd));
#endif
    if (nbytes > 0) {
        JointTestState(cmd);
    }
    test_joint(homingPos, homingPos.size(), 0.05, r_pos);
    /////////////////////////////////////////////////////////
    for (int i=0; i<homingPos.size(); i++) {
        r_vel[i] = DEG2RAD(20)*1000;
    }
    // send position and velocity reference
    boards_ctrl->set_velocity(r_vel, sizeof(r_vel));
    boards_ctrl->set_position(r_pos, sizeof(r_pos));
    // log
    log_elem.ts = get_time_ns() - g_tStart;
    for (int i=0; i<MAX_DSP_BOARDS; i++) {
        log_elem.pos[i] = r_pos[i];
    }
    log_buff.push_back(log_elem);
}
#endif

void _shut(void * _)
{
    home_body();
    sleep(2);
    boards_ctrl->start_stop_control(false);
    boards_ctrl->stop_rx_udp();
    boards_ctrl->start_stop_bc_boards(false);

    delete imu;
    delete boards_ctrl;
    dump_master_log();
    close(fd_info);
    close(fd_data);
}


///////////////////////////////////////////////////////////////////////////////
//
///////////////////////////////////////////////////////////////////////////////


void * test_boards_thread(void * arg) {
    thread_info_t info;
    info.name = "boards_ctrl";
    info.term_mode = END_JOIN;
    info.period.period = {0,1000};

#if DEMO_TYPE==1
    rt_hook_t       hook = { _init, _loop_joint, _shut};
#elif DEMO_TYPE==2
    rt_hook_t       hook = { _init, _loop_walking, _shut};
#endif

#if RT_ENV
    return rt_periodic_thread((void*)&info, &hook);
#else
    return nrt_thread((void*)&info, &hook);
#endif
}
