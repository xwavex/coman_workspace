#include <math.h>
# define N 31
#include <iostream>
#include "bezier.hh"
#include <fstream>

using namespace std;

void init_pos(double time, double Pos_sens0[], double Qfinal[], double Pos_sens[], double Vel_sens[], double tauDes[], double vals[], int whichComan)
{
    double y[N];  // desired output to be driven to zero
    double dy[N]; // derivative of the desired output to be driven to zero
    double Kp[N], Kd[N], I[N];
    double ALPHA = 0.8;
    double pos_des;
    double vel_des;
#ifndef REAL_ROBOT
    ALPHA = 1;

    for (int i = 0; i < N; i++)
    {
        // pos_des = qinit<vector> + *(error -> sensor - qinit)*e^(-alpha*time)
        pos_des = Qfinal[i]+(Pos_sens0[i]-Qfinal[i])*exp(-ALPHA * time);
        vel_des = -ALPHA*(Pos_sens0[i]-Qfinal[i])*exp(-ALPHA * time);
        y[i] = Pos_sens[i]-pos_des;
        dy[i] = Vel_sens[i]-vel_des;
    }


    // Lower Body
    
    for (int i = 0; i < 23; i++)
    {
        Kp[i] = 300;
        Kd[i] = 3;
    }

    Kp[23] = 5;
    Kd[23] = 0;
    Kp[24] = 5;
    Kd[24] = 0;
    Kp[25] = 5;
    Kd[25] = 0;
    Kp[26] = 0;
    Kd[26] = 0;
    Kp[27] = 5;
    Kd[27] = 0;
    Kp[28] = 5;
    Kd[28] = 0;
    Kp[29] = 0;
    Kd[29] = 0;
    Kp[30] = 0;
    Kd[30] = 0;
    Kp[31] = 0;
    Kd[31] = 0;

    
    for (int i = 0; i < N; i++)
    {
        tauDes[i] = -Kp[i]*(y[i])-Kd[i]*(dy[i]);
    }

#else

    for (int i = 0; i < N; i++)
    {
        pos_des = Qfinal[i]+(Pos_sens0[i]-Qfinal[i])*exp(-ALPHA * time);
        vel_des = -ALPHA*(Pos_sens0[i]-Qfinal[i])*exp(-ALPHA * time);
        y[i] = Pos_sens[i]-pos_des;
        dy[i] = Vel_sens[i]-vel_des;
    }

    for (int i = 0; i < N; i++)
    {
        Kp[i] = 500;
        Kd[i] = 15;
    }

    for (int i = 23; i < N; i++)
        Kd[i] = 0;
    Kd[23] = 10;
    Kp[23] = 100;
    if (whichComan == 2){
        Kp[24] = 50; //coman 1 -50
        Kp[29] = 50;
        Kp[30] = 100;
    }
    else
    {
        Kp[24] = -50;
        Kp[29] = -50;
        Kp[30] = -100;
    }
    if (whichComan == 0){
        Kp[24] = 0;
        // std::cout << "Attention!! WhichComan is not set!! Kp[24] was set to zero!" << std::endl; // *** Is going into this statement even though whichcoman is 1
    }
    Kp[25] = 50;
    Kp[26] = 100;
    Kd[26] = 10;
    Kp[27] = 50;
    Kp[28] = 50;



    double VelZero[N];
    for (int i = 0; i < N; i++)
    {
        VelZero[i] = 0;
    }
    VelZero[24] = 1;
    VelZero[25] = 1;
    VelZero[27] = 1;
    VelZero[28] = 1;
    VelZero[29] = 1;
    VelZero[30] = 1;

    double MAX_VOLTAGE[N];
    for (int i = 0; i < 15; i++)
    {
        MAX_VOLTAGE[i] = 12;
    }
    for (int i = 15; i < 23; i++)
    {
        MAX_VOLTAGE[i] = 6;
    }
    for (int i = 23; i < N; i++)
    {
        MAX_VOLTAGE[i] = 6;
    }

    double MAX_CURRENT[N];
    for (int i = 0; i < 15; i++)
    {
        MAX_CURRENT[i] = 8;
    }
    for (int i = 15; i < 23; i++)
    {
        MAX_CURRENT[i] = 8;
    }
    for (int i = 23; i < N; i++)
    {
        MAX_CURRENT[i] = 8;
    }

    for (int i = 0; i < N; i++)
    {
        tauDes[i] = -Kp[i]*(y[i])-Kd[i]*(dy[i]);
        I[i] = (tauDes[i]+0*0.25+0)/(0.055*100);
        I[i] = I[i]>MAX_CURRENT[i] ? MAX_CURRENT[i]: (I[i]<-MAX_CURRENT[i] ? -MAX_CURRENT[i] : I[i]);
        vals[i] = 0.055*100*(1-VelZero[i])*Vel_sens[i]+2.5*I[i]; // 100 is the grear ratio
        vals[i] = vals[i]>MAX_VOLTAGE[i] ? MAX_VOLTAGE[i]: (vals[i]<-MAX_VOLTAGE[i] ? -MAX_VOLTAGE[i] : vals[i]);
    }

//    cout << vals[24] << " " << Pos_sens[24] << " " << Vel_sens[24] << endl;

#endif
}


