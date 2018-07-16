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
#include <boost/tokenizer.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <boost/numeric/ublas/lu.hpp>
#include <boost/numeric/ublas/io.hpp>

#include <fstream>
#include <string>
#include <vector>
#include <iostream>

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


extern int loop;

///////////////////////////////////////////////////////////////////////////////
// Globals
///////////////////////////////////////////////////////////////////////////////

int fd_info, fd_data;
uint64_t  g_tStart = 0;
boost::circular_buffer<log_ctrl_t> log_buff(LOG_SIZE);

Boards_ctrl * boards_ctrl = new Boards_ctrl("config.yaml");
#if RT_ENV
//rt_imu_3DM_GX3_25  * imu;
#else
//nrt_imu_3DM_GX3_25  * imu;
#endif

// home position in degree (homing position of the real Arm)
// pay attention joint 2 and 4 has 0 as fisical limit
std::vector<float> homePos = {
// for 0,0,0,90:
   24.8, 4.2,  -7.6,  97.5,
//  1,  2,  3,  4
};

// homing position of the simulated Arm (defined based on the DH parameters, Matlab model)
std::vector<float> Sim_homing = {
// for 0,0,0,90:
   0.0, 0.0,  0.0,  90.0,
//  1,  2,  3,  4
};

//#define CURR_BOARD_ID 4

// boards ID to control
std::vector<int> boards_set  = {1,2,3,4};// { 1,2,3,4 };

std::vector<std::vector<int>> trj_values;
std::vector<std::vector<int>> piezo;

using namespace boost::numeric::ublas;

/* Matrix inversion routine.
 Uses lu_factorize and lu_substitute in uBLAS to invert a matrix */
template<class T>
bool InvertMatrix(const matrix<T>& input, matrix<T>& inverse)
{
	typedef permutation_matrix<std::size_t> pmatrix;

	// create a working copy of the input
	matrix<T> A(input);

	// create a permutation matrix for the LU-factorization
	pmatrix pm(A.size1());

	// perform LU-factorization
	int res = lu_factorize(A, pm);
	if (res != 0)
		return false;

	// create identity matrix of "inverse"
	inverse.assign(identity_matrix<T> (A.size1()));

	// backsubstitute to get the inverse
	lu_substitute(A, pm, inverse);

	return true;
}
///////// vector definitions for the inverse kinematic calculation
matrix<double> jacobian (3, 4);
vector<double> cart_pos (3);
////////////// DH Constants of the Arm
static const double LSEH = 95.96e-3;
static const double LEW = 358.8e-3+0.1;
static const double LBS = 261.6e-3;
static const double LSE = 430.43e-3;

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
        //for (int j=0; j<MAX_DSP_BOARDS; j++) {
        //    fprintf(fp,"%d\t", log_buff[i].pos[j]);
        //}
        fprintf(fp,"\n");
    }
    fclose(fp);
}

void load_ref(std::string filename, std::vector<std::vector<int>> &csv_values) {

    std::fstream file(filename, std::ios::in);

    if (file) {
        typedef boost::tokenizer< boost::char_separator<char> > Tokenizer;
        boost::char_separator<char> sep("\t");
        std::string line;

        while (getline(file, line)) {

            Tokenizer info(line, sep); // tokenize the line of data
            std::vector<int> values;

            for (Tokenizer::iterator it = info.begin(); it != info.end(); ++it) {
                // convert data into double value, and store
                values.push_back(atoi(it->c_str()));
            }
            // store array of values
            csv_values.push_back(values);
        }

    } else {
        std::cerr << "Error: Unable to open file " << filename << std::endl;
    }
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

static void body_homing(int pos[], short vel[], short tor[], bc_data_t * curr_bc_data) {

    if ( curr_bc_data ) {

        for (int j=0; j<boards_set.size(); j++) {
            pos[boards_set[j]-1] = curr_bc_data[boards_set[j]-1].mc_bc_data.Position;
            //DPRINTF("homing %d ", pos[boards_set[j]-1]);
        }
        //DPRINTF("\n");
    } else {

        for (int i=0; i<homePos.size(); i++) {
            pos[i] = DEG2mRAD(homePos[i]);
        }
    }

    for (int i=0; i<homePos.size(); i++) {
        vel[i] = DEG2RAD(25)*1000;
    }
    for (int i=0; i<homePos.size(); i++) {
        tor[i] = 0;
    }

}

static void trajectory_NVB(int pos[], short vel[], short tor[], int damp[], bc_data_t * curr_bc_data, uint64_t t_UserKey_ns) {
    //  DO NOT TOUCH IT !!!     DO NOT TOUCH IT !!!     DO NOT TOUCH IT !!!     DO NOT TOUCH IT !!!
    //  DO NOT TOUCH IT !!!     DO NOT TOUCH IT !!!     DO NOT TOUCH IT !!!     DO NOT TOUCH IT !!!
    ///////////////////////////////////////////////////////////////////////////////////////////////
    //  THIS FUNCTION IS VALID JUST FOR 4 DOF ARM !!!
    ///////////////////////////////////////////////////////////////////////////////////////////////
    // Defining the runing time since the command is started
    uint64_t tNow = get_time_ns();
    uint64_t dt_ns = (tNow - t_UserKey_ns);
    double run_time = dt_ns/1.0e9;
    int Dmaping_Voltage = 0;
    //
    static std::vector<int> deltaq(4);
    static std::vector<int> initial_def(4);
    static std::vector<int> initial_pos(4);
    static std::vector<double> initial_cart_pos(3);
    static std::vector<double> relative_cart_pos(3);
    static std::vector<double> relative_cart_desired_pos(3);
    static std::vector<double> cart_desired_pos(3);
    static std::vector<double> desired_pos(4);
    static std::vector<double> cart_desired_pos_previous_sample(3);
    static std::vector<double> desired_pos_previous_sample(4);
    matrix<double> jacobian_transpose (4, 3);
    matrix<double> jjt (3, 3);
    matrix<double> jacobian_inverse (4, 3);
    matrix<double> inv_jj (3, 3);
    vector<double> delta_cart_desired_pos(3);
    vector<double> delta_desired_pos(4);
/////////////////////////////////////////////////////////
    if (dt_ns < 10000)
    {
        for (int j=0; j<4; j++) {
            //// initial positions of joints
            initial_pos[j] = curr_bc_data[j].mc_bc_data.Position;
            //// initialization of the cartesian position of previous sample time
            desired_pos_previous_sample[j] = curr_bc_data[j].mc_bc_data.Position;
        }

        for (int j=0; j<3; j++) {
            //// initial Cartesian position
            initial_cart_pos[j] = cart_pos[j];
            //// initialization of the cartesian position of previous sample time
            cart_desired_pos_previous_sample[j] = cart_pos[j];
        }
    }
    ///////// The first second is dedicated for initialization of the initialization of the loading of the springs
    if (dt_ns < 1.0e9)
    {
        for (int j=0; j<4; j++) {
        //// initial deflection of joints
        initial_def[j] = curr_bc_data[j].mc_bc_data.Deflection;
        }
    }
    ///////// Definition of relative desired cartesian position
    static double trajectory_final_time = 5.0; //// in second
    if (run_time < trajectory_final_time)
    {
        relative_cart_desired_pos[0]=0.0;
        relative_cart_desired_pos[1]=0.0+0.3*run_time/trajectory_final_time;
        relative_cart_desired_pos[2]=0.0+0.4*run_time/trajectory_final_time;
    }
    ////// calculation of desired cartesian position
    for (int j=0; j<3; j++) {
        cart_desired_pos[j] = relative_cart_desired_pos[j]+initial_cart_pos[j];
        delta_cart_desired_pos[j]=(cart_desired_pos[j]-cart_desired_pos_previous_sample[j])*1.0e5;
    }
    /////// Calculation of inverse jacobian (pesudo invserse)
    jacobian_transpose = trans(jacobian);
    jjt = prod(jacobian,jacobian_transpose);
    InvertMatrix(jjt, inv_jj);
    jacobian_inverse = prod(jacobian_transpose,inv_jj);
    ///////
    delta_desired_pos = prod(jacobian_inverse,delta_cart_desired_pos);
    for (int j=0; j<4; j++) {
        //// the desired position calculation
        desired_pos[j] = desired_pos_previous_sample[j]+(delta_desired_pos[j]);
        //// the desired position of previous sample time
        desired_pos_previous_sample[j] = desired_pos[j];
    }
    //// the desired cartesian position of previous sample time
    for (int j=0; j<3; j++) {
        cart_desired_pos_previous_sample[j] = cart_desired_pos[j];
    }
    /////////////////////////////////////////////////////////
//    std::cout << run_time << "\t"<< jacobian<< "\t"<< delta_desired_pos<< "\t"<< delta_cart_desired_pos<< "\t"<< curr_bc_data[0].mc_bc_data.Position<< "\t"<< curr_bc_data[1].mc_bc_data.Position<<"\t"<< curr_bc_data[2].mc_bc_data.Position<<"\t"<< curr_bc_data[3].mc_bc_data.Position<< std::endl;



     //std::cout << run_time << "\t"<< deltaq[0]<< "\t"<< deltaq[1]<< "\t"<< deltaq[2]<< "\t"<< deltaq[3]<< std::endl;


    /////////////////////////////////////////////////////////
    // desired Damping (Voltage!)
    for (int j=0; j<boards_set.size(); j++) {
//                pos[boards_set[j]-1] =initial_pos[boards_set[j]-1];
                damp[boards_set[j]-1] =Dmaping_Voltage;
    }
    // desired position
    for (int j=0; j<boards_set.size(); j++) {
//                pos[boards_set[j]-1] =initial_pos[boards_set[j]-1];
                pos[boards_set[j]-1] =desired_pos[boards_set[j]-1];
    }
    // desired velocity
    for (int i=0; i<MAX_DSP_BOARDS; i++) {
        vel[i] = DEG2RAD(50)*1000;
    }
    // desired torque
    for (int i=0; i<MAX_DSP_BOARDS; i++) {
        tor[i] = 0;
    }

}

static void trajectory_stop(int pos[], short vel[], short tor[], bc_data_t * curr_bc_data, uint64_t t_UserKey_ns) {

    // Defining the runing time since the command is started
    uint64_t tNow = get_time_ns();
    uint64_t dt_ns = tNow - t_UserKey_ns;
    //
    static std::vector<int> initial_pos(4);
    // finding the initial joint position
    if (dt_ns < 10000)
    {
        for (int j=0; j<boards_set.size(); j++) {
            initial_pos[boards_set[j]-1] = curr_bc_data[boards_set[j]-1].mc_bc_data.Position;
        }
    }
    // desired position
    for (int j=0; j<boards_set.size(); j++) {
                pos[boards_set[j]-1] =initial_pos[boards_set[j]-1];
    }
    // desired velocity
    for (int i=0; i<MAX_DSP_BOARDS; i++) {
        vel[i] = DEG2RAD(50)*1000;
    }
    // desired torque
    for (int i=0; i<MAX_DSP_BOARDS; i++) {
        tor[i] = 0;
    }

}

static void trajectory_homing(int pos[], short vel[], short tor[], bc_data_t * curr_bc_data, uint64_t t_UserKey_ns) {

    // Defining the runing time since the command is started
    uint64_t tNow = get_time_ns();
    uint64_t dt_ns = tNow - t_UserKey_ns;
    //
    static std::vector<int> initial_pos(4);
    static std::vector<int> stepsize(4);
    static std::vector<int> trj(4);
    // finding the initial joint position
    if (dt_ns < 10000)
    {
        for (int j=0; j<boards_set.size(); j++) {
            initial_pos[boards_set[j]-1] = curr_bc_data[boards_set[j]-1].mc_bc_data.Position;
            stepsize[boards_set[j]-1] =100000*M_PI/180.0*homePos[boards_set[j]-1] - initial_pos[boards_set[j]-1];
        }
    }
    // relative desired position
    if(dt_ns<1e9) {
        for (int j=0; j<boards_set.size(); j++) {
            trj[boards_set[j]-1] = stepsize[boards_set[j]-1]*(dt_ns*1.0e-9);
        }
    }
    // desired position
    for (int j=0; j<boards_set.size(); j++) {
                pos[boards_set[j]-1] =initial_pos[boards_set[j]-1]+trj[boards_set[j]-1];
    }
    // desired velocity
    for (int i=0; i<MAX_DSP_BOARDS; i++) {
        vel[i] = DEG2RAD(50)*1000;
    }
    // desired torque
    for (int i=0; i<MAX_DSP_BOARDS; i++) {
        tor[i] = 0;
    }

}

static void trajectory(int pos[], short vel[], short tor[], bc_data_t * curr_bc_data, uint64_t t_UserKey_ns) {

    // Defining the runing time since the command is started
    uint64_t tNow = get_time_ns();
    uint64_t dt_ns = tNow - t_UserKey_ns;
    //
    static std::vector<int> initial_pos(4);
    static std::vector<double> trj(4);
    // finding the initial joint position
    if (dt_ns < 10000)
    {
        for (int j=0; j<boards_set.size(); j++) {
            initial_pos[boards_set[j]-1] = curr_bc_data[boards_set[j]-1].mc_bc_data.Position;
        }
    }
    // relative desired position (in degrees)
    static double freq_Hz = 2;
    trj[0]= 0;
    trj[1]= 0;
    trj[2]= 5.0*sin((2.0 * M_PI * freq_Hz * dt_ns)/1e9);
    trj[3]= 0;

    // desired position
    for (int j=0; j<boards_set.size(); j++) {
    pos[boards_set[j]-1] =initial_pos[boards_set[j]-1]+DEG2mRAD(trj[boards_set[j]-1]);
    }

    // desired velocity
    for (int i=0; i<MAX_DSP_BOARDS; i++) {
        vel[i] = DEG2RAD(50)*1000;
    }
    // desired torque
    for (int i=0; i<MAX_DSP_BOARDS; i++) {
        tor[i] = 0;
    }

}

static void trajectory2(int pos[], short vel[], short tor[], int damp[], bc_data_t * curr_bc_data, uint64_t t_UserKey_ns) {

    // Defining the runing time since the command is started

    // Definition of Desired Trajectory:

    // Defining the runing time since the command is started
    uint64_t tNow = get_time_ns();
    uint64_t dt_ns = tNow - t_UserKey_ns;
    //
    static std::vector<int> initial_pos(4);
    // finding the initial joint position
    if (dt_ns < 10000)
    {
        for (int j=0; j<boards_set.size(); j++) {
            initial_pos[boards_set[j]-1] = curr_bc_data[boards_set[j]-1].mc_bc_data.Position;
        }
    }
    // desired position
    for (int j=0; j<boards_set.size(); j++) {
                pos[boards_set[j]-1] =initial_pos[boards_set[j]-1];
    }
    // desired velocity
    for (int i=0; i<MAX_DSP_BOARDS; i++) {
        vel[i] = DEG2RAD(50)*1000;
    }
    // desired torque
    for (int i=0; i<MAX_DSP_BOARDS; i++) {
        tor[i] = 0;
    }
    // damping definition
    int _damp = 0;
    /*
    if ((dt_ns>=1e9) && (dt_ns<2e9))             _damp=16383;
    else if ((dt_ns>=3e9) && (dt_ns<4e9))       _damp = 16383.0*(dt_ns/1.0e9-3.0);                         //mVolts to piezo
    else if ((dt_ns>=5e9) && (dt_ns<6e9))       _damp = 163830.0*(dt_ns/1.0e9-5.0);
    else if ((dt_ns>=7e9) && (dt_ns<8e9))       _damp = 1638300.0*(dt_ns/1.0e9-7.0);
    else if ((dt_ns>=9e9) && (dt_ns<10e9))       _damp = 16383.0*(dt_ns/1.0e9-9.0)*(dt_ns/1.0e9-9.0);                         //mVolts to piezo
    else if ((dt_ns>=11e9) && (dt_ns<12e9))       _damp = 163830.0*(dt_ns/1.0e9-11.0)*(dt_ns/1.0e9-11.0);
    else if ((dt_ns>=13e9) && (dt_ns<14e9))       _damp = 1638300.0*(dt_ns/1.0e9-13.0)*(dt_ns/1.0e9-13.0);
    else if ((dt_ns>=15e9) && (dt_ns<16e9))      _damp = 16383.0*(dt_ns/1.0e9-15.0)*(dt_ns/1.0e9-15.0)*(dt_ns/1.0e9-15.0)*((dt_ns/1.0e9-15.0)*((dt_ns/1.0e9-15.0)*6.0 - 15.0) + 10.0);
    else if ((dt_ns>=17e9) && (dt_ns<18e9))      _damp = 4*16383.0*(dt_ns/1.0e9-17.0)*(dt_ns/1.0e9-17.0)*(dt_ns/1.0e9-17.0)*((dt_ns/1.0e9-17.0)*((dt_ns/1.0e9-17.0)*6.0 - 15.0) + 10.0);
    else if ((dt_ns>=19e9) && (dt_ns<20e9))      _damp = 10*16383.0*(dt_ns/1.0e9-19.0)*(dt_ns/1.0e9-19.0)*(dt_ns/1.0e9-19.0)*((dt_ns/1.0e9-19.0)*((dt_ns/1.0e9-19.0)*6.0 - 15.0) + 10.0);
    */


/*    if ((dt_ns>=1e9) && (dt_ns<2e9))   _damp=16383;
    else if ((dt_ns>=3e9) && (dt_ns<4e9))      _damp = 16383.0*(dt_ns/1.0e9-3.0)*(dt_ns/1.0e9-3.0);                         //mVolts to piezo
    else if ((dt_ns>=4e9) && (dt_ns<5e9))      _damp = 16383.0;
    else if ((dt_ns>=5e9) && (dt_ns<10e9))     _damp = 8192.0+8191.0*sin((M_PI*dt_ns*(dt_ns/1.0e9-5.0))/5.0e9);
    else if ((dt_ns>=10e9) && (dt_ns<11e9))    _damp = 8192.0;
    else if ((dt_ns>=11e9) && (dt_ns<12e9))    _damp = 8192.0-8192.0*(dt_ns/1.0e9-11.0);
*/

    _damp=0;
    if (dt_ns<1e9)   _damp=17000;
    else if ((dt_ns>=2e9) && (dt_ns<3e9))     _damp =5000;                         //mVolts to piezo
    else if ((dt_ns>=4e9) && (dt_ns<6e9))     _damp =10000;                         //mVolts to piezo
    else if ((dt_ns>=7e9) && (dt_ns<10e9))    _damp =17000;                         //mVolts to piezo
    else if ((dt_ns>=20e9) && (dt_ns<25e9))   _damp =16383.0/3.0*(dt_ns/1.0e9-20.0);                         //mVolts to piezo
    else if ((dt_ns>=25e9) && (dt_ns<30e9))   _damp =16383.0/3.0;                         //mVolts to piezo
    else if ((dt_ns>=30e9) && (dt_ns<35e9))   _damp =16383.0/3.0+16383.0/3.0*(dt_ns/1.0e9-30.0);                         //mVolts to piezo
    else if ((dt_ns>=35e9) && (dt_ns<40e9))   _damp =2.0*16383.0/3.0;                         //mVolts to piezo
    else if ((dt_ns>=40e9) && (dt_ns<45e9))   _damp =2.0*16383.0/3.0+16383.0/3.0*(dt_ns/1.0e9-40.0);
    else if ((dt_ns>=50e9) && (dt_ns<55e9))   _damp =16383;                         //mVolts to piezo



    // !!! pay attention
    //((McBoard*)(*boards_ctrl)[CURR_BOARD_ID])->set_damping((int)damp);
    /*
    for (int j=0; j<boards_set.size(); j++) {
        damp[boards_set[j]-1] = _damp;
    }
    */
    damp[0]= _damp;
    damp[1]= 0.0;
    damp[2]= 0.0;
    damp[3]= 0.0;
}

static void trajectory_file(int pos[], short vel[], short tor[], int damp[], bc_data_t * curr_bc_data, uint64_t t_UserKey_ns) {

    static std::vector<std::vector<int>>::const_iterator it = trj_values.begin();
    static std::vector<std::vector<int>>::const_iterator pit = piezo.begin();
    static int end_of_trj;
    uint64_t tNow = get_time_ns();

    if ( tNow - t_UserKey_ns < 1e9 && end_of_trj) {
        it = trj_values.begin();
        end_of_trj = 0;
    }

    if ( it == trj_values.end() ) {
        end_of_trj = 1;
        return;
    }


    const std::vector<int> values(*it);
    // !!! first column in file is time !!!!
    for (int j=0; j<boards_set.size(); j++) {
        pos[boards_set[j]-1] = values[boards_set[j]] + (int)(DEG2mRAD(homePos[boards_set[j]-1])-DEG2mRAD(Sim_homing[boards_set[j]-1]));
    }
// for printing the output:
// std::cout << pos[0] << "\t" << pos[1] << "\t" << pos[2] << "\t" << pos[3] << std::endl;

    if ( pit != piezo.end() ) {

        const std::vector<int> _damp(*pit);
        for (int j=0; j<boards_set.size(); j++) {
            //((McBoard*)(*boards_ctrl)[boards_set[j]])->set_damping(damp[boards_set[j]]);
            damp[boards_set[j]-1] = _damp[boards_set[j]-1];
        }

        pit++;
   }


    // desired velocity
    for (int i=0; i<MAX_DSP_BOARDS; i++) {
        vel[i] = DEG2RAD(50)*1000;
    }

    // desired torque
    for (int i=0; i<MAX_DSP_BOARDS; i++) {
        tor[i] = 0;
    }

    it++;

}


static void calc_jacobian(double q1, double q2, double q3, double q4 ) {


    jacobian(0,0) = LEW*sin(q4)*(-cos(q3)*sin(q1)+sin(q2)*sin(q3)*cos(q1))+LSE*cos(q1)*cos(q2)-LSEH*cos(q3)*sin(q1)+LEW*cos(q4)*cos(q1)*cos(q2)+LSEH*sin(q2)*sin(q3)*cos(q1);
    jacobian(0,1) = LEW*sin(q4)*sin(q1)*cos(q2)*sin(q3)-LSE*sin(q1)*sin(q2)-LEW*cos(q4)*sin(q1)*sin(q2)+LSEH*sin(q1)*cos(q2)*sin(q3);
    jacobian(0,2) = LEW*sin(q4)*(-sin(q3)*cos(q1)+sin(q1)*sin(q2)*cos(q3))-LSEH*sin(q3)*cos(q1)+LSEH*sin(q1)*sin(q2)*cos(q3);
    jacobian(0,3) = LEW*cos(q4)*(cos(q3)*cos(q1)+sin(q1)*sin(q2)*sin(q3))-LEW*sin(q4)*sin(q1)*cos(q2);
    jacobian(1,0) = 0.0;
    jacobian(1,1) = -LEW*cos(q2)*cos(q4)-LSE*cos(q2)-LSEH*sin(q2)*sin(q3)-LEW*sin(q4)*sin(q3)*sin(q2);
    jacobian(1,2) = LSEH*cos(q3)*cos(q2)+LEW*sin(q4)*cos(q3)*cos(q2);
    jacobian(1,3) = LEW*sin(q2)*sin(q4)+LEW*cos(q4)*sin(q3)*cos(q2);
    jacobian(2,0) = LSE*sin(q1)*cos(q2)-LEW*sin(q4)*(-cos(q3)*cos(q1)-sin(q1)*sin(q2)*sin(q3))+LSEH*cos(q3)*cos(q1)+LEW*cos(q4)*sin(q1)*cos(q2)+LSEH*sin(q1)*sin(q2)*sin(q3);
    jacobian(2,1) = LSE*cos(q1)*sin(q2)-LEW*sin(q4)*sin(q3)*cos(q2)*cos(q1)+LEW*cos(q4)*cos(q1)*sin(q2)-LSEH*sin(q3)*cos(q2)*cos(q1);
    jacobian(2,2) = -LEW*sin(q4)*(sin(q3)*sin(q1)+sin(q2)*cos(q3)*cos(q1))-LSEH*sin(q3)*sin(q1)-LSEH*sin(q2)*cos(q3)*cos(q1);
    jacobian(2,3) = -LEW*cos(q4)*(-cos(q3)*sin(q1)+sin(q2)*sin(q3)*cos(q1))+LEW*sin(q4)*cos(q1)*cos(q2);

    //std::cout << jacobian << std::endl;

}


static void calc_cpos(double q1, double q2, double q3, double q4 ) {

    cart_pos[0] = LEW*sin(q4)*(cos(q3)*cos(q1)+sin(q1)*sin(q2)*sin(q3))+LSE*sin(q1)*cos(q2)+LSEH*cos(q3)*cos(q1)+LEW*cos(q4)*sin(q1)*cos(q2)+LSEH*sin(q1)*sin(q2)*sin(q3);
    cart_pos[1] = -LEW*sin(q2)*cos(q4)-LSE*sin(q2)-LBS+LSEH*sin(q3)*cos(q2)+LEW*sin(q4)*sin(q3)*cos(q2);
    cart_pos[2] = -LSE*cos(q1)*cos(q2)-LEW*sin(q4)*(-cos(q3)*sin(q1)+sin(q2)*sin(q3)*cos(q1))+LSEH*cos(q3)*sin(q1)-LEW*cos(q4)*cos(q1)*cos(q2)-LSEH*sin(q2)*sin(q3)*cos(q1);

    //std::cout << cart_pos << std::endl;

}

///////////////////////////////////////////////////////////////////////////////
//
///////////////////////////////////////////////////////////////////////////////


void _init(void * _)
{
    int         r_pos[MAX_DSP_BOARDS];
    short       r_vel[MAX_DSP_BOARDS];
    short       r_tor[MAX_DSP_BOARDS];
    bc_data_t   bc_data[MAX_DSP_BOARDS];

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

    load_ref("data_export.txt", trj_values);
    DPRINTF("Load trajetory .... %d\n", trj_values.size());
    load_ref("piezo_data.txt", piezo);
    DPRINTF("Load piezo .... %d\n", piezo.size());
#if 0
    for (std::vector<std::vector<double>>::const_iterator it = trj_values.begin(); it != trj_values.end(); ++it) {
        const std::vector<double>& values = *it;
        for (std::vector<double>::const_iterator it2 = values.begin(); it2 != values.end(); ++it2) {
            std::cout << *it2 << " ";
        }
        std::cout << std::endl;
    }
#endif

    boards_ctrl->init();
    //
    DPRINTF("Scan for active boards ....\n");
    numActive = boards_ctrl->scan4active();
    DPRINTF("Found %d boards\n", numActive);

    // use config.yaml settings
    boards_ctrl->configure_boards();

    boards_ctrl->test();

    boards_ctrl->start_stop_control(false);
    g_tStart = get_time_ns();
    // tell to ALL dps to start broadcast data
    boards_ctrl->start_stop_bc_boards(true);
    // wait ....
    usleep(1000);
    // for running all joints
    //boards_ctrl->start_stop_control(true);

    DPRINTF("Start control ...\n");
    // for not running the controller, the following line should be commented
    start_control_body(boards_set);

    boards_ctrl->get_bc_data(bc_data);
    body_homing(r_pos, r_vel, r_tor, bc_data);
    boards_ctrl->set_position(r_pos, sizeof(r_pos));
    boards_ctrl->set_velocity(r_vel, sizeof(r_vel));
    //DPRINTF("%d %d %d %d\n", r_pos[0], r_pos[1], r_pos[2], r_pos[3]);


}

void  _loop(void * _)
{
    // initialized to zero
    static std::vector<int> delta_pos(MAX_DSP_BOARDS, 0);
    static int      pos_incr[MAX_DSP_BOARDS];
    static int      damp_incr[MAX_DSP_BOARDS];
    static int      r_pos[MAX_DSP_BOARDS];
    static int      r_pos_prev[MAX_DSP_BOARDS];
    static int      r_stif[MAX_DSP_BOARDS];
    static int      r_damp[MAX_DSP_BOARDS];
    static short    r_vel[MAX_DSP_BOARDS];
    static short    r_tor[MAX_DSP_BOARDS];

    bc_data_t   bc_data[MAX_DSP_BOARDS];
    log_ctrl_t  log_elem;
    int nbytes;
    char cmd;

    static uint16_t dac_output = 0;
    static pid_gains_t  force_pid;
    //static int r_damp;
    static int trj_flag;
    short reply_abs_pos;
    static uint64_t  g_tUserKey;

    ///// step size vector defining the change in position of the joints when keys command
    delta_pos[0] = DEG2mRAD(0);
    delta_pos[1] = DEG2mRAD(0);
    delta_pos[2] = DEG2mRAD(0);
    delta_pos[3] = DEG2mRAD(10);

    /////////////////////////////////////////////////////////
    // Sense
    //boards_ctrl->get_sync_data((void*)0);
    boards_ctrl->get_bc_data(bc_data);
    //(*boards_ctrl)[0]->
    // idx --> bId-1
    //bc_data[0].mc_bc_data.Position;

    /////////////////////////////////////////////////////////
    static double q1,q2, q3, q4;    ///// definition of joint variables
    q1 = bc_data[0].mc_bc_data.Position /1e5-(homePos[0]-Sim_homing[0])*M_PI/180;
    q2 = bc_data[1].mc_bc_data.Position /1e5-(homePos[1]-Sim_homing[1])*M_PI/180;
    q3 = bc_data[2].mc_bc_data.Position /1e5-(homePos[2]-Sim_homing[2])*M_PI/180;
    q4 = bc_data[3].mc_bc_data.Position /1e5-(homePos[3]-Sim_homing[3])*M_PI/180;
    /////////   Jacobian Maatrix and End-effector positoin Calculation
    calc_jacobian(q1,q2,q3,q4);
    calc_cpos(q1,q2,q3,q4);
//    std::cout << cart_pos << std::endl;
    /////////////////////////////////////////////////////////
    body_homing(r_pos, r_vel, r_tor, bc_data);
    /////////////////////////////////////////////////////////
    // NON-BLOCKING read char from pipe or cross domain socket
#if RT_ENV
    nbytes = recvfrom(fd_info, (void*)&cmd, sizeof(cmd), MSG_DONTWAIT, NULL, 0);
#else
    // NON-BLOCKING
    nbytes = read(fd_info, (void*)&cmd, sizeof(cmd));
#endif
    if (nbytes > 0) {
        // process char
        DPRINTF("RX %c\n", cmd);
        switch ( cmd ) {
            case 'a':
                //dac_output += 1000;
                //pos_incr[CURR_BOARD_ID-1] += DEG2mRAD(10);
                // desired velocity
                //r_vel[CURR_BOARD_ID-1] = DEG2RAD(25)*1000;
                for (int j=0; j<boards_set.size(); j++) {
                    pos_incr[boards_set[j]-1] += delta_pos[boards_set[j]-1];
                    r_vel[boards_set[j]-1] = DEG2RAD(25)*1000;
                    //DPRINTF("pos_incr %d vel %d\n", pos_incr[boards_set[j]-1], r_vel[boards_set[j]-1]);
                }
                break;
            case 'z':
                //dac_output -= 1000;
                //pos_incr[CURR_BOARD_ID-1] -= DEG2mRAD(10);
                // desired velocity
                //r_vel[CURR_BOARD_ID-1] = DEG2RAD(25)*1000;
                for (int j=0; j<boards_set.size(); j++) {
                    pos_incr[boards_set[j]-1] -= delta_pos[boards_set[j]-1];
                    r_vel[boards_set[j]-1] = DEG2RAD(25)*1000;
                    //DPRINTF("pos_incr %d vel %d\n", pos_incr[boards_set[j]-1], r_vel[boards_set[j]-1]);
                }
                break;
            case 's':
                force_pid.p += 5;
                break;
            case 'x':
                force_pid.p -= 5;
                break;
            case 'd':
                force_pid.i += 1;
                break;
            case 'c':
                force_pid.i -= 1;
                break;
            case 'f':
                force_pid.d += 1;
                break;
            case 'v':
                force_pid.d -= 1;
                break;
            case 'k':
                //r_damp += 17000;
                //if (r_damp > 50000) r_damp = 50000;
                //((McBoard*)(*boards_ctrl)[CURR_BOARD_ID])->set_damping(r_damp);
                //for (int j=0; j<boards_set.size(); j++) {
                //    ((McBoard*)(*boards_ctrl)[boards_set[j]])->set_damping(r_damp);
                //}
                for (int j=0; j<boards_set.size(); j++) {
                    damp_incr[boards_set[j]-1] += 17000;
                    if (damp_incr[boards_set[j]-1] > 50000) damp_incr[boards_set[j]] = 50000;
                    r_damp[boards_set[j]-1] = damp_incr[boards_set[j]-1];
                }
                break;
            case 'm':
                //r_damp -= 17000;
                //if (r_damp < 0) r_damp = 0;
                //((McBoard*)(*boards_ctrl)[CURR_BOARD_ID])->set_damping(r_damp);
                //for (int j=0; j<boards_set.size(); j++) {
                //    ((McBoard*)(*boards_ctrl)[boards_set[j]])->set_damping(r_damp);
                //}
                for (int j=0; j<boards_set.size(); j++) {
                    damp_incr[boards_set[j]-1] -= 17000;
                    if (damp_incr[boards_set[j]-1] < 0) damp_incr[boards_set[j]-1] = 0;
                    r_damp[boards_set[j]-1] = damp_incr[boards_set[j]-1];
                }
                break;
/*
            case 'p':
                if ( ! trj_flag) trj_flag = 1;
                else trj_flag = 0;
                g_tUserKey = get_time_ns();
                pos_incr[CURR_BOARD_ID-1] = 0;
                break;
*/
            case 'q':
                //(*boards_ctrl)[CURR_BOARD_ID]->setItem(CALIBRATE_TORQUE_OFFSET, NULL, 0);
                for (int j=0; j<boards_set.size(); j++) {
                    ((McBoard*)(*boards_ctrl)[boards_set[j]])->setItem(CALIBRATE_TORQUE_OFFSET, NULL, 0);
                }
                break;

            case '0':
                trj_flag = 0;
                break;
            case '1':   // for using commands
                trj_flag = 1;
                //pos_incr[CURR_BOARD_ID-1] = 0;
                for (int j=0; j<boards_set.size(); j++) {
                    pos_incr[boards_set[j]-1] = DEG2mRAD(homePos[boards_set[j]-1]);
                }
                break;
            case '2':   // for using trajectory 1

                g_tUserKey = get_time_ns();
                trj_flag = 2;
                break;
            case '3':   // for using trajectory 2
                g_tUserKey = get_time_ns();
                trj_flag = 3;
                break;  // trajectory from the file
            case '4':
                g_tUserKey = get_time_ns();
                trj_flag = 4;
                break;
            case '5':   // homing
                g_tUserKey = get_time_ns();
                trj_flag = 5;
                break;
            case '6':   // holding
                g_tUserKey = get_time_ns();
                trj_flag = 6;
                break;
            case '7':   // NAVVAB
                g_tUserKey = get_time_ns();
                trj_flag = 7;
                break;
            default:
                break;
        }
    }
    /////////////////////////////////////////////////////////


    //((McBoard*)(*boards_ctrl)[3])->test_piezo_out(dac_output);
#define DO_FORCE_LOOP_TEST 0
#if DO_FORCE_LOOP_TEST
    /////////////////////////////   for testing the force loop
    // use SET_PID_GAINS with gains_set 4 to set desired damping
    /*
    bc_data[CURR_BOARD_ID-1].mc_bc_data.Position = ((McBoard*)(*boards_ctrl)[CURR_BOARD_ID])->test_force_loop(force_pid);
    r_pos[CURR_BOARD_ID-1] = bc_data[CURR_BOARD_ID-1].mc_bc_data.Position;
    */
    // the above two lines are changed to the following lines for being able to read the position

    //line down sends force/voltage reference
    r_pos[CURR_BOARD_ID-1] = ((McBoard*)(*boards_ctrl)[CURR_BOARD_ID])->test_force_loop(force_pid);
#else

    switch (trj_flag) {
        case 0:
            body_homing(r_pos, r_vel, r_tor, bc_data);
            break;
        case 1:
        //5 degrees increment (decrement), see case a, z
            for (int j=0; j<boards_set.size(); j++) {
                r_pos[boards_set[j]-1] = pos_incr[boards_set[j]-1];
            }
            break;
        case 2:
        //damping change (see case)
            trajectory(r_pos, r_vel, r_tor, bc_data, g_tUserKey);
            break;
        case 3:
        //COMMENT THIS!
            trajectory2(r_pos, r_vel, r_tor, r_damp, bc_data, g_tUserKey);
            break;
        case 4:
            trajectory_file(r_pos, r_vel, r_tor, r_damp, bc_data, g_tUserKey);
            break;
        case 5:
            trajectory_homing(r_pos, r_vel, r_tor, bc_data, g_tUserKey);
            break;
        case 6:
            trajectory_stop(r_pos, r_vel, r_tor, bc_data, g_tUserKey);
            break;
        case 7:
            trajectory_NVB(r_pos, r_vel, r_tor, r_damp, bc_data, g_tUserKey);
            break;
        default:
            break;
    }

/*
    if (trj_flag) {
        trajectory(r_pos, r_vel, r_tor, g_tUserKey);
    } else {
        r_pos[CURR_BOARD_ID-1] +=  pos_incr[CURR_BOARD_ID-1];
    }
*/

        boards_ctrl->set_stiffness_damping(r_stif, r_damp, boards_set.size());
        boards_ctrl->set_velocity(r_vel, sizeof(r_vel));
        boards_ctrl->set_position(r_pos, sizeof(r_pos));
        memcpy((void*)r_pos_prev, (void*)r_pos, sizeof(r_pos));


    //boards_ctrl->test();
    //DPRINTF("r_pos %d\n", r_pos[CURR_BOARD_ID-1]);

#endif
    /////////////////////////////////////////////////////////
    // write to pipe or cross domain socket ALL bc data
    write(fd_data, (void*)&bc_data, sizeof(bc_data));
    /////////////////////////////////////////////////////////




    /////////////////////////////////////////////////////////
    // log
    log_elem.ts = get_time_ns() - g_tStart;
    for (int i=0; i<MAX_DSP_BOARDS; i++) {
        log_elem.pos[i] = r_pos[i];
    }
    log_buff.push_back(log_elem);
    /////////////////////////////////////////////////////////

}


void _shut(void * _)
{
    //struct timespec ts = {3,0};
    //clock_nanosleep(CLOCK_REALTIME, 0, &ts, NULL);
    sleep(3);

    boards_ctrl->start_stop_control(false);
    boards_ctrl->stop_rx_udp();
    boards_ctrl->start_stop_bc_boards(false);
    boards_ctrl->start_stop_bc_boards(false);

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
    info.period.period = {0,1000};

    rt_hook_t       hook = { _init, _loop, _shut};

#if RT_ENV
    return rt_periodic_thread((void*)&info, &hook);
#else
    return nrt_thread((void*)&info, &hook);
#endif
}


