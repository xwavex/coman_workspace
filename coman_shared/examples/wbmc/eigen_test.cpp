#include <Eigen/Dense>
#include <iostream>
#include <cmath>

#include <definitions.h>
#include <utils.h>
#include <wbmc_func.h>

#include <Boards_ctrl_wbmc.h>

using namespace Eigen;
using namespace std;

#define JOINT_NUM   23
#define DIM         JOINT_NUM+6

// loop time in sec
//const float Ts = 0;

const float SafeMar = 0.1;


Matrix<int,23,29> Sq = Matrix<int,23,29>::Zero();

const float epsq = M_PI/180.0;
const float epsqd = M_PI/180.0/1;

const Vector3f Ref_LMom = Vector3f::Zero();
const Vector3f Ref_AMom = Vector3f::Zero();


float min_effort_test(void);

float min_effort(int i,
                 Matrix<float,29,1>& q,
                 Matrix<float,29,1>& sinq,
                 Matrix<float,29,1>& cosq,
                 float epsq, float tau_GC, float effort, float k);

void tau_GravityComp(Matrix<float,29,1>& sinq,
                     Matrix<float,29,1>& cosq,
                     Matrix<float,29,1>& h,
                     Matrix<float,23,29>& proj,
                     Matrix<float,29,1>& tau_GC);

void tau_GravityComp_test(Matrix<float,29,1>& q);


void Boards_ctrl_wbmc::control_loop_TEST(void) {

    static Matrix<float,DIM,1>  q;
    //q.Zero();
    q << 0.13115,-0.0149173,0.487819,0,0,0,0.27665,0.03041,0.00101,-0.10381,-0.05102,0.03788,0.29519,0.04264,0.01797,0.01598,-0.03818,-0.02403,0.039,0.20016,-1e-05,-1.57343,1.03457,0.14598,0,1.5914,-1.5038,-0.067,-0.08711;
    tau_GravityComp_test(q);
    return;
}

/* Control loop */
void Boards_ctrl_wbmc::control_loop(void) {

    uint64_t tNow, dt;
    int nbytes;

    Matrix<float,DIM,1> limNeg;
    limNeg << DEG2RAD(-10000), DEG2RAD(-10000), DEG2RAD(-10000), DEG2RAD(-10000), DEG2RAD(-10000), DEG2RAD(-10000),
                                       DEG2RAD(-110-(20)), DEG2RAD(-60), DEG2RAD(-50), DEG2RAD(0), DEG2RAD(-35), DEG2RAD(-70), DEG2RAD(-110-(20)),
                                       DEG2RAD(-25), DEG2RAD(-50), DEG2RAD(0), DEG2RAD(-35), DEG2RAD(-70), DEG2RAD(-30), DEG2RAD(-20-(20)),
                                       DEG2RAD(-80), DEG2RAD(-195), DEG2RAD(-170), DEG2RAD(-90), DEG2RAD(-135), DEG2RAD(-195), DEG2RAD(-18),
                                       DEG2RAD(-90), DEG2RAD(-135);

    Matrix<float,DIM,1>   limPos;
    limPos << DEG2RAD(10000), DEG2RAD(10000), DEG2RAD(10000), DEG2RAD(10000), DEG2RAD(10000), DEG2RAD(10000),
                                       DEG2RAD(45-(20)), DEG2RAD(25), DEG2RAD(50), DEG2RAD(110), DEG2RAD(35), DEG2RAD(50), DEG2RAD(45-(20)),
                                       DEG2RAD(60), DEG2RAD(50), DEG2RAD(110), DEG2RAD(35), DEG2RAD(50), DEG2RAD(30), DEG2RAD(50-(20)),
                                       DEG2RAD(80), DEG2RAD(95), DEG2RAD(18), DEG2RAD(90), DEG2RAD(0), DEG2RAD(95), DEG2RAD(170),
                                       DEG2RAD(90), DEG2RAD(0);

    Matrix<float,DIM,1>     rangeJ = limPos - limNeg;
    Matrix<float,DIM,1>     thNeg = limNeg + SafeMar * rangeJ;
    Matrix<float,DIM,1>     thPos = limPos - SafeMar * rangeJ;


    static Matrix<float,DIM,1>  q,
                                qd,
                                qd_eps,
                                h,
                                sinq,
                                cosq,
                                tau_JLimP,
                                tau_JLimD,
                                tau_ree,
                                tau_lee;

    static Vector3f         LMom, AMom,
                            RHand_act, RHandd_act, LHand_act, LHandd_act;

    static Vector3f     LMom_eps, AMom_eps;

    static float                   NormLM, NormAM, effort;

    static Matrix<float,23,29>     proj;

    static Matrix<float,23,23>     M;
    static Matrix<float, 3,29>     J_LMcom, J_AMcom;
    static Matrix<float, 3,29>     J_RH;
    static Matrix<float, 3,29>     J_LH;


    static Matrix<float,DIM,1>     tau_GC,
                                   tau_MomJ,
                                   tau_LMomCOM,
                                   tau_AMomCOM,
                                   tau_MEff,
                                   tau_JLimPD,
                                   tau_RH,
                                   tau_LH,
                                   tau;


    static uint64_t ti_ree, T_ree, Thold_ree, Trel_ree, ti_lee, T_lee, Thold_lee, Trel_lee;
    static float xi_ree, yi_ree, zi_ree, xi_lee, yi_lee, zi_lee, xf_ree, yf_ree, zf_ree, xf_lee, yf_lee, zf_lee, xref_ree, yref_ree, zref_ree, xref_lee, yref_lee, zref_lee, xdref_ree, ydref_ree, zdref_ree, xdref_lee, ydref_lee, zdref_lee;
    static float xe_ree, ye_ree, ze_ree;

    static Vector3f F_ree, F_lee;

    /* Derived variables */
    float tf_ree = ti_ree + T_ree;
    float tf2_ree = tf_ree + Thold_ree;
    float tf3_ree = tf2_ree + Trel_ree;

    float tf_lee = ti_lee + T_lee;
    float tf2_lee = tf_lee + Thold_lee;
    float tf3_lee = tf2_lee + Trel_lee;

    // q position
    // q 1..3 compute later with dirDyn
    // q 4..6 imu data
    // q 7..29 mc board

    // qd velocity
    // qd 1..6 imu data
    // qd 7..29 mc board

    //cout << Matrix<int,23,23>(Matrix<int,23,1>::Ones().asDiagonal()) << endl;
    Sq.block<23,23>(0,6) = Matrix<int,23,23>::Identity(); // (Matrix<int,23,1>::Ones().asDiagonal());
    //cout << "Sq =" << Sq << endl;

    q.head(3) = Matrix<float,3,1>::Zero();
    q.segment(3,3) = Matrix<float,3,1>::Zero();
    qd.head(6) = Matrix<float,6,1>::Zero();

#ifdef TEST
    q.tail(DIM-6)  = Matrix<float,DIM-6,1>::Random();
    qd.tail(DIM-6) = Matrix<float,DIM-6,1>::Random();
#else
    //for (int i=6; i<DIM; i++) {
    //    q(i)  = (float)_ts_bc_data[i-6].raw_bc_data.mc_bc_data.Position / 1e5;
    //    qd(i) = (float)_ts_bc_data[i-6].raw_bc_data.mc_bc_data.Velocity / 1e3;
    //}

    q(6)  = (float)_ts_bc_data[4-1].raw_bc_data.mc_bc_data.Position / 1e5;
    qd(6) = (float)_ts_bc_data[4-1].raw_bc_data.mc_bc_data.Velocity / 1e3;

    q(7)  = (float)_ts_bc_data[6-1].raw_bc_data.mc_bc_data.Position / 1e5;
    qd(7) = (float)_ts_bc_data[6-1].raw_bc_data.mc_bc_data.Velocity / 1e3;

    q(8)  = (float)_ts_bc_data[7-1].raw_bc_data.mc_bc_data.Position / 1e5;
    qd(8) = (float)_ts_bc_data[7-1].raw_bc_data.mc_bc_data.Velocity / 1e3;

    q(9)  = (float)_ts_bc_data[8-1].raw_bc_data.mc_bc_data.Position / 1e5;
    qd(9) = (float)_ts_bc_data[8-1].raw_bc_data.mc_bc_data.Velocity / 1e3;

    q(10)  = (float)_ts_bc_data[10-1].raw_bc_data.mc_bc_data.Position / 1e5;
    qd(10) = (float)_ts_bc_data[10-1].raw_bc_data.mc_bc_data.Velocity / 1e3;

    q(11)  = (float)_ts_bc_data[9-1].raw_bc_data.mc_bc_data.Position / 1e5;
    qd(11) = (float)_ts_bc_data[9-1].raw_bc_data.mc_bc_data.Velocity / 1e3;

    q(12)  = (float)_ts_bc_data[5-1].raw_bc_data.mc_bc_data.Position / 1e5;
    qd(12) = (float)_ts_bc_data[5-1].raw_bc_data.mc_bc_data.Velocity / 1e3;

    q(13)  = (float)_ts_bc_data[11-1].raw_bc_data.mc_bc_data.Position / 1e5;
    qd(13) = (float)_ts_bc_data[11-1].raw_bc_data.mc_bc_data.Velocity / 1e3;

    q(14)  = (float)_ts_bc_data[12-1].raw_bc_data.mc_bc_data.Position / 1e5;
    qd(14) = (float)_ts_bc_data[12-1].raw_bc_data.mc_bc_data.Velocity / 1e3;

    q(15)  = (float)_ts_bc_data[13-1].raw_bc_data.mc_bc_data.Position / 1e5;
    qd(15) = (float)_ts_bc_data[13-1].raw_bc_data.mc_bc_data.Velocity / 1e3;

    q(16)  = (float)_ts_bc_data[15-1].raw_bc_data.mc_bc_data.Position / 1e5;
    qd(16) = (float)_ts_bc_data[15-1].raw_bc_data.mc_bc_data.Velocity / 1e3;

    q(17)  = (float)_ts_bc_data[14-1].raw_bc_data.mc_bc_data.Position / 1e5;
    qd(17) = (float)_ts_bc_data[14-1].raw_bc_data.mc_bc_data.Velocity / 1e3;

    q(18)  = (float)_ts_bc_data[3-1].raw_bc_data.mc_bc_data.Position / 1e5;
    qd(18) = (float)_ts_bc_data[3-1].raw_bc_data.mc_bc_data.Velocity / 1e3;

    q(19)  = DEG2RAD(20) + (float)_ts_bc_data[2-1].raw_bc_data.mc_bc_data.Position / 1e5;
    qd(19) = (float)_ts_bc_data[2-1].raw_bc_data.mc_bc_data.Velocity / 1e3;

    q(20)  = (float)_ts_bc_data[1-1].raw_bc_data.mc_bc_data.Position / 1e5;
    qd(20) = (float)_ts_bc_data[1-1].raw_bc_data.mc_bc_data.Velocity / 1e3;

    q(21)  = (float)_ts_bc_data[16-1].raw_bc_data.mc_bc_data.Position / 1e5;
    qd(21) = (float)_ts_bc_data[16-1].raw_bc_data.mc_bc_data.Velocity / 1e3;

    q(22)  = (float)_ts_bc_data[17-1].raw_bc_data.mc_bc_data.Position / 1e5;
    qd(22) = (float)_ts_bc_data[17-1].raw_bc_data.mc_bc_data.Velocity / 1e3;

    q(23)  = (float)_ts_bc_data[18-1].raw_bc_data.mc_bc_data.Position / 1e5;
    qd(23) = (float)_ts_bc_data[18-1].raw_bc_data.mc_bc_data.Velocity / 1e3;

    q(24)  = (float)_ts_bc_data[19-1].raw_bc_data.mc_bc_data.Position / 1e5;
    qd(24) = (float)_ts_bc_data[19-1].raw_bc_data.mc_bc_data.Velocity / 1e3;

    q(25)  = (float)_ts_bc_data[20-1].raw_bc_data.mc_bc_data.Position / 1e5;
    qd(25) = (float)_ts_bc_data[20-1].raw_bc_data.mc_bc_data.Velocity / 1e3;

    q(26)  = (float)_ts_bc_data[21-1].raw_bc_data.mc_bc_data.Position / 1e5;
    qd(26) = (float)_ts_bc_data[21-1].raw_bc_data.mc_bc_data.Velocity / 1e3;

    q(27)  = (float)_ts_bc_data[22-1].raw_bc_data.mc_bc_data.Position / 1e5;
    qd(27) = (float)_ts_bc_data[22-1].raw_bc_data.mc_bc_data.Velocity / 1e3;

    q(28)  = (float)_ts_bc_data[23-1].raw_bc_data.mc_bc_data.Position / 1e5;
    qd(28) = (float)_ts_bc_data[23-1].raw_bc_data.mc_bc_data.Velocity / 1e3;

#endif


    // offset tra Robotran e COMAN solo per le spalle !!!!
    // bId 17 q position is bId-1 + 6
    q(22) -= M_PI/2;
    q(26) += M_PI/2;

    // log q and qd
    q_log.push_back(q);
    qd_log.push_back(qd);

    sinq.array() = q.array().sin();
    cosq.array() = q.array().cos();
    //cout << "sinq =" << sinq << endl;
    //cout << "cosq =" << cosq << endl;

    q.head(3) = DirDyn(sinq, cosq);
    //cout << q.head(3) << endl;

    tNow = get_time_ns();
    /* Gravity compensation */
    for (int i=0; i<29; i++ ) {
        h[i] = tau_func_ptr[i](sinq, cosq); //(s4,...,s29,c4,...,c29);
    }
    dt = get_time_ns()-tNow;
    //DPRINTF("tau %.6f\n", (float)dt/1e6);


    tNow = get_time_ns();
    for (int i=0; i<23; i++ ) {
        for (int j=0; j<23; j++ ) {
            M(i,j) = inertia_func_ptr[i][j](sinq, cosq); //(s4,...,s29,c4,...,c29);
        }
    }
    dt = get_time_ns()-tNow;
    //DPRINTF("inertia %.6f\n", (float)dt/1e6);

    /////////////////////////////////////////////////////////////////////////////////////
    //
    /////////////////////////////////////////////////////////////////////////////////////

    tau_GravityComp(sinq, cosq, h, proj, tau_GC);
    effort = tau_GC.transpose() * tau_GC;

    /////////////////////////////////////////////////////////////////////////////////////
    //
    /////////////////////////////////////////////////////////////////////////////////////

    //  q,sinq,cosq,epsq,tau_GC,effort,k[4];
    min_effort_args_t min_effort_args;
    Map<Matrix<float,29,1>>(min_effort_args.q) = q;
    Map<Matrix<float,29,1>>(min_effort_args.sinq) = sinq;
    Map<Matrix<float,29,1>>(min_effort_args.cosq) = cosq;
    Map<Matrix<float,29,1>>(min_effort_args.tau_GC) = tau_GC;
    min_effort_args.epsq = epsq;
    min_effort_args.effort = effort;
    min_effort_args.k = k[4];
    min_effort_args.ts = get_time_ns();
#if 1
    nbytes = parallel_calc_WR((void*)&min_effort_args, sizeof(min_effort_args));
    //DPRINTF("parallel_calc_WR %d\n", nbytes);
#endif
    /////////////////////////////////////////////////////////////////////////////////////
    //
    /////////////////////////////////////////////////////////////////////////////////////

    tau_MomJ.head(6) = Matrix<float,6,1>::Zero();
    tau_MomJ.tail(23) = k[1] * qd.tail(23).transpose() * M;

    /* LMom@COM and AMom@COM */
    J_lm(sinq, cosq, J_LMcom); //(s4,...,s29,c4,...,c29)
    //!!!  = 6x29 x 29x1
    LMom = J_LMcom * qd;
    NormLM = (Ref_LMom - LMom).squaredNorm();
    J_am(sinq, cosq, J_AMcom); //(s4,...,s29,c4,...,c29)
    //!!!  = 6x29 x 29x1
    AMom = J_AMcom * qd;
    NormAM = (Ref_AMom - AMom).squaredNorm();

    for(int i=6; i<29; i++) {
        qd_eps = qd;
        qd_eps[i] = qd[i] + epsqd;
        // i-esima col = 3x29 * 29x1  LMom_eps[:][i] = J_LMcom * qd_eps
        LMom_eps = J_LMcom * qd_eps;
        // i-esimo elem 29x1
        tau_LMomCOM(i) = k[2] * ((Ref_LMom - LMom_eps).squaredNorm() - NormLM) / epsqd;
        // i-esima col = 3x29 * 29x1
        AMom_eps = J_AMcom * qd_eps;
        // i-esimo elem
        tau_AMomCOM(i) = k[3] * ((Ref_AMom - AMom_eps).squaredNorm() - NormAM) / epsqd;
    }

    //////////////////////////////////////////////////////////////////////////////////////
    // MinEff
    //////////////////////////////////////////////////////////////////////////////////////

    tNow = get_time_ns();
    float min_effort_res[29];
#if 1
    // BLOCKING ... 
    nbytes = parallel_calc_RO((void*)min_effort_res, sizeof(min_effort_res));
    //DPRINTF("parallel_calc_RO %d\n", nbytes);
    tau_MEff = Map<Matrix<float,29,1>>(min_effort_res);
#else
    for (int i=6; i<29; i++) {
        tau_MEff(i) = min_effort(i,q,sinq,cosq,epsq,tau_GC(i),effort,k[4]);
    }
#endif
    dt = get_time_ns()-tNow;
    //DPRINTF("minEff %.6f\n", (float)dt/1e6);

    /////////////////////////////////////////////////////////////////////////////////////
    // Joint limits
    /////////////////////////////////////////////////////////////////////////////////////

    tNow = get_time_ns();
    for(int i=6; i<29; i++) {
        if(q[i]<thNeg[i]) {
            tau_JLimP[i] = k[5] / pow(limNeg[i] - thNeg[i], 2) * pow(q[i]-thNeg[i], 2);
            tau_JLimD[i] = -k[6] * qd[i];
            tau_JLimPD[i] = tau_JLimP[i] + tau_JLimD[i];

            if(tau_JLimPD[i] < 0) {
                tau_JLimPD[i] = 0;
            }
        }

        if(q[i]>thPos[i]) {
            tau_JLimP[i] = -k[5] / pow(limPos[i] - thPos[i], 2) * pow(q[i] - thPos[i], 2);
            tau_JLimD[i] = -k[6] * qd[i];
            tau_JLimPD[i] = tau_JLimP[i] + tau_JLimD[i];
            if(tau_JLimPD[i] > 0) {
                tau_JLimPD[i] = 0;
            }
        }
        if((q[i]>=thNeg[i]) && (q[i]<=thPos[i])) {
            tau_JLimPD[i] = 0;
        }
    }
    dt = get_time_ns()-tNow;
    //DPRINTF("jointLim %.6f\n", (float)dt/1e6);

    /////////////////////////////////////////////////////////////////////////////////////
    // Right end-effector
    /////////////////////////////////////////////////////////////////////////////////////

    dt = get_time_ns(); - g_tStart;

    if (tNow >= ti_ree && tNow <= tf_ree) {
        RHand_act = P_RHand(q, sinq, cosq); //(s4,...,s29,c4,...,c29)
        J_RHand(sinq, cosq, J_RH); //(s4,...,s29,c4,...,c29)
        RHandd_act = J_RH * qd;

        if(tNow == ti_ree) {
            xi_ree = RHand_act[1];
            yi_ree = RHand_act[2];
            zi_ree = RHand_act[3];
        }

        xref_ree = -2 / pow(T_ree,3) * (xf_ree - xi_ree) * pow(dt-ti_ree,3) + 3 / pow(T_ree,2) * (xf_ree - xi_ree) * pow(dt-ti_ree,2) + xi_ree;
        yref_ree = -2 / pow(T_ree,3) * (yf_ree - yi_ree) * pow(dt-ti_ree,3) + 3 / pow(T_ree,2) * (yf_ree - yi_ree) * pow(dt-ti_ree,2) + yi_ree;
        zref_ree = -2 / pow(T_ree,3) * (zf_ree - zi_ree) * pow(dt-ti_ree,3) + 3 / pow(T_ree,2) * (zf_ree - zi_ree) * pow(dt-ti_ree,2) + zi_ree;

        xdref_ree = -6 / pow(T_ree,3) * (xf_ree - xi_ree) * pow(dt-ti_ree,2) + 6 / pow(T_ree,2) * (xf_ree - xi_ree) * (dt-ti_ree);
        ydref_ree = -6 / pow(T_ree,3) * (yf_ree - yi_ree) * pow(dt-ti_ree,2) + 6 / pow(T_ree,2) * (yf_ree - yi_ree) * (dt-ti_ree);
        zdref_ree = -6 / pow(T_ree,3) * (zf_ree - zi_ree) * pow(dt-ti_ree,2) + 6 / pow(T_ree,2) * (zf_ree - zi_ree) * (dt-ti_ree);

        F_ree[1] = k[7] * (xref_ree - RHand_act[1]) + k[8] * (xdref_ree - RHandd_act[1]);
        F_ree[2] = k[7] * (yref_ree - RHand_act[2]) + k[8] * (ydref_ree - RHandd_act[2]);
        F_ree[3] = k[7] * (zref_ree - RHand_act[3]) + k[8] * (zdref_ree - RHandd_act[3]);
    }


    if( tNow > tf_ree && tNow <= tf2_ree ) {
        F_ree[1] = k[7] * (xf_ree - RHand_act[1]) + k[8] * (- RHandd_act[1]);
        F_ree[2] = k[7] * (yf_ree - RHand_act[2]) + k[8] * (- RHandd_act[2]);
        F_ree[3] = k[7] * (zf_ree - RHand_act[3]) + k[8] * (- RHandd_act[3]);
    }

    if( dt > tf2_ree && dt <= tf3_ree) {
        F_ree[1] = (tf3_ree - dt) / (tf3_ree - tf2_ree) * (k[7] * (xf_ree - RHand_act[1]) + k[8] * (- RHandd_act[1]));
        F_ree[2] = (tf3_ree - dt) / (tf3_ree - tf2_ree) * (k[7] * (yf_ree - RHand_act[2]) + k[8] * (- RHandd_act[2]));
        F_ree[3] = (tf3_ree - dt) / (tf3_ree - tf2_ree) * (k[7] * (zf_ree - RHand_act[3]) + k[8] * (- RHandd_act[3]));
    }

    tau_ree = J_RH.transpose() * F_ree;

    if( dt < ti_ree|| dt > tf3_ree ) {
        tau_ree.Zero();
    }

    tau_RH.head(6) = Matrix<float,6,1>::Zero();
    tau_RH.tail(23) = proj * tau_ree;


    /////////////////////////////////////////////////////////////////////////////////////
    // Left end-effector
    /////////////////////////////////////////////////////////////////////////////////////

    if ( dt >= ti_lee && dt <= tf_lee ) {
        LHand_act = P_RHand(q, sinq, cosq); //(s4,...,s29,c4,...,c29)
        J_LHand(sinq, cosq, J_RH); //(s4,...,s29,c4,...,c29)
        LHandd_act = J_LH * qd;

        if(dt ==ti_lee) {
            xi_lee = LHand_act[1];
            yi_lee = LHand_act[2];
            zi_lee = LHand_act[3];
        }

        xref_lee = -2 / pow(T_lee,3) * (xf_lee - xi_lee) * pow(dt-ti_lee,3) + 3 / pow(T_lee,2) * (xf_lee - xi_lee) * pow(dt-ti_lee,2) + xi_lee;
        yref_lee = -2 / pow(T_lee,3) * (yf_lee - yi_lee) * pow(dt-ti_lee,3) + 3 / pow(T_lee,2) * (yf_lee - yi_lee) * pow(dt-ti_lee,2) + yi_lee;
        zref_lee = -2 / pow(T_lee,3) * (zf_lee - zi_lee) * pow(dt-ti_lee,3) + 3 / pow(T_lee,2) * (zf_lee - zi_lee) * pow(dt-ti_lee,2) + zi_lee;

        xdref_lee = -6 / pow(T_lee,3) * (xf_lee - xi_lee) * pow(dt-ti_lee,2) + 6 / pow(T_lee,2) * (xf_lee - xi_lee) * (dt-ti_lee);
        ydref_lee = -6 / pow(T_lee,3) * (yf_lee - yi_lee) * pow(dt-ti_lee,2) + 6 / pow(T_lee,2) * (yf_lee - yi_lee) * (dt-ti_lee);
        zdref_lee = -6 / pow(T_lee,3) * (zf_lee - zi_lee) * pow(dt-ti_lee,2) + 6 / pow(T_lee,2) * (zf_lee - zi_lee) * (dt-ti_lee);

        F_lee[1] = k[7] * (xref_lee - LHand_act[1]) + k[8] * (xdref_lee - LHandd_act[1]);
        F_lee[2] = k[7] * (yref_lee - LHand_act[2]) + k[8] * (ydref_lee - LHandd_act[2]);
        F_lee[3] = k[7] * (zref_lee - LHand_act[3]) + k[8] * (zdref_lee - LHandd_act[3]);

    }

    if( dt > tf_lee && dt <= tf2_lee) {
        F_lee[1] = k[7] * (xf_lee - LHand_act[1]) + k[8] * (- LHandd_act[1]);
        F_lee[2] = k[7] * (yf_lee - LHand_act[2]) + k[8] * (- LHandd_act[2]);
        F_lee[3] = k[7] * (zf_lee - LHand_act[3]) + k[8] * (- LHandd_act[3]);

    }

    if( dt > tf2_lee && dt <= tf3_lee) {
        F_lee[1] = (tf3_lee - dt) / (tf3_lee - tf2_lee) * (k[7] * (xf_lee - LHand_act[1]) + k[8] * (- LHandd_act[1]));
        F_lee[2] = (tf3_lee - dt) / (tf3_lee - tf2_lee) * (k[7] * (yf_lee - LHand_act[2]) + k[8] * (- LHandd_act[2]));
        F_lee[3] = (tf3_lee - dt) / (tf3_lee - tf2_lee) * (k[7] * (zf_lee - LHand_act[3]) + k[8] * (- LHandd_act[3]));
    }

    tau_lee = J_LH.transpose() * F_lee;

    if( dt < ti_lee|| dt > tf3_lee ) {
        tau_lee.Zero();
    }

    tau_LH.head(6) = Matrix<float,6,1>::Zero();
    tau_LH.tail(23) = proj * tau_lee;


    tau = tau_GC*k[0] + tau_MomJ + tau_LMomCOM + tau_AMomCOM + tau_MEff + tau_JLimPD + tau_RH + tau_LH;

    // from Nm to mNm
    tau *= 1e3;

    tau_log.push_back(tau);
    tau_GC_log.push_back(tau_GC*k[0]);
    tau_MinEff_log.push_back(tau_MEff);
    tau_Mom_log.push_back(tau_MomJ);

    _tor[0] = 0; //tau(21-1);
    _tor[1] = tau(20-1);
    _tor[2] = tau(19-1);
    _tor[3] = 0; //tau(7-1);
    _tor[4] = 0; //tau(13-1);
    _tor[5] = 0; //tau(8-1);
    _tor[6] = 0; //tau(9-1);
    _tor[7] = 0; //tau(10-1);
    _tor[8] = 0; //tau(12-1);
    _tor[9] = 0; //tau(11-1);
    _tor[10] = 0; //tau(14-1);
    _tor[11] = 0; //tau(15-1);
    _tor[12] = 0; //tau(16-1);
    _tor[13] = 0; //tau(18-1);
    _tor[14] = 0; //tau(17-1);
    _tor[15] = tau(22-1);
    _tor[16] = tau(23-1);
    _tor[17] = 0; //tau(24-1);
    _tor[18] = 0; //tau(25-1);
    _tor[19] = 0; //tau(26-1);
    _tor[20] = 0; //tau(27-1);
    _tor[21] = 0; //tau(28-1);
    _tor[22] = 0; //tau(29-1);

    cout << "tau_GC =" << tau_GC(18)*k[0] << " " << tau_GC(19)*k[0] << endl;
    cout << "tau_MomJ =" << tau_MomJ(18) << " " << tau_MomJ(19) << endl;
    cout << "tau_MinEff =" << tau_MEff(18) << " " << tau_MEff(19) << endl;

    cout << "tau =" << tau(18) << " " << tau(19) << endl;


}
