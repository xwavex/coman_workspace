#include <wbmc_func.h>
#include <iostream>

using namespace Eigen;
using namespace std;

#define sign(x)     (((x) > 0.0) ? 1 : -1)

// A non-zero value (true) if the sign of x is negative; and zero (false) otherwise.
// signbit(x)

static float __sign (float x) {
    //if ( signbit(x) )
    if ( x < 0.0 )
        return -1;
    return 1;
}


/**
 *
 * @param Matrix<float,29,1> sinq
 *
 * @param Matrix<float,29,1> cosq
 *
 * @param Matrix<float,29,1> h
 *
 * @return Matrix<float,29,1> tau_GC
 *
 *
 *  */
void tau_GravityComp(Matrix<float,29,1>& sinq,
                     Matrix<float,29,1>& cosq,
                     Matrix<float,29,1>& h,
                     Matrix<float,23,29>& proj,
                     Matrix<float,29,1>& tau_GC) {

    static Matrix<float,29,17>     Vc;
    static Matrix<float,23,17>     SqV;
    static Matrix<float,17,23>     SqVpinv;

    V(sinq, cosq, Vc);
    // SqV 23x17 Vc 29x17 -- take last 23 rows of Vc
    SqV = Vc.bottomRows(23); // [7:29][1:17]
    // pseudo inverse ... inv(A'*A)*A'
    SqVpinv = (SqV.transpose() * SqV).inverse() * SqV.transpose();
    //23x29= 23x17      17x23     23x17 17x29
    //proj = SqVpinv' * SqVpinv * SqV * Vc'
    proj = SqVpinv.transpose() * SqVpinv * SqV * Vc.transpose();
    //cout << "Vc " << Vc << endl;
    //cout << "SqVpinv " << SqVpinv << endl;

    tau_GC.head(6) = Matrix<float,6,1>::Zero();
    tau_GC.tail(23) = proj * h;

}

/**
 *
 * @param int i
 *
 * @param Matrix<float,29,1> q
 *
 * @param Matrix<float,29,1> sinq
 *
 * @param Matrix<float,29,1> cosq
 *
 * @param epsq float
 *
 * @param tau_GC float
 *
 * @param k float
 *
 *  */
float min_effort(int i,
                 Matrix<float,29,1>& q,
                 Matrix<float,29,1>& sinq,
                 Matrix<float,29,1>& cosq,
                 float epsq, float tau_GC, float effort, float k) {

    static Matrix<float,29,1>   h_eps,
                                tau_GC_eps,
                                sinq_eps,
                                cosq_eps;

    static Matrix<float,29,17>     Vc_eps;
    static Matrix<float,23,17>     SqV_eps;
    static Matrix<float,17,23>     SqVpinv_eps;
    static Matrix<float,23,29>     proj_eps;

    static float effort_eps, tau_min_effort;

    sinq_eps = sinq;
    sinq_eps(i) = sin(q(i)+epsq);

    cosq_eps = cosq;
    cosq_eps(i) = cos(q(i)+epsq);


    // 7 10 12 13 16 18 ===> 6 9 11 12 15 17
    if(i==6||i==9||i==11||i==12||i==15||i==17) {
        sinq_eps(4) = sin(q(4)-epsq);
        cosq_eps(4) = cos(q(4)-epsq);
    }

    // 8 11 14 17 ===> 7 10 13 16
    if(i==7||i==10||i==13||i==16) {
        sinq_eps(3) = sin(q(3)-epsq);
        cosq_eps(3) = cos(q(3)-epsq);
    }

    // 9 15 ===> 8 14
    if(i==8||i==14) {
        sinq_eps(5) = sin(q(5)-epsq);
        cosq_eps(5) = cos(q(5)-epsq);
    }

    /* Gravity compensation */
    for (int j=0; j<29; j++ ) {
        // !!!!! s4...c29 from sinq_eps cosq_eps
        h_eps[j] = tau_func_ptr[j](sinq_eps, cosq_eps);
    }

    tau_GravityComp(sinq_eps, cosq_eps, h_eps, proj_eps, tau_GC_eps);

    effort_eps = tau_GC_eps.transpose() * tau_GC_eps;
    tau_min_effort = - k * sign(effort_eps - effort) * pow(tau_GC,2);
    //fprintf(stderr,"%d %f %f   %f  %f \n", i, effort_eps, effort, epsq, effort - effort_eps);

    return tau_min_effort;

}


float min_effort_test(void) {

    Matrix<float,29,1> q = Matrix<float,29,1>::Random();
    return min_effort(6,q,q,q,0,0,0,0);
}

void tau_GravityComp_test(Matrix<float,29,1>& q) {

    static Matrix<float,29,1>   h,
                                tau_GC,
                                sinq,
                                cosq;
    static Matrix<float,23,29>  proj;

    sinq.array() = q.array().sin();
    cosq.array() = q.array().cos();

    for (int j=0; j<29; j++ ) {
        // !!!!! s4...c29 from sinq_eps cosq_eps
        h[j] = tau_func_ptr[j](sinq, cosq);
    }

    tau_GravityComp(sinq, cosq, h, proj, tau_GC);

    //cout << "Proj " << proj << endl;
    //cout << "Tau_GC " << tau_GC.transpose() << endl;
}
