/*
    Updated date: 16 Nov 2012
    Copyright (C) 2012 Italian Institute of Technology

    Developer:
        Zhibin Li, email: zhibin.li@iit.it
    Description: this file provides the forward/inverse kinematics for the COMAN arms and legs.
    More detailed information can be found in the accompanying documentation.
*/

#include <cmath>
#include "Matrix.h"

#define RADTODEG(x)  x*180.0/M_PI
#define DEGTORAD(x)  x*M_PI/180.0

const double upperleg=0.22580000;
const double lowerleg=0.2010000;
const double ankle_height = 0.0793;
const double fullleg=upperleg+lowerleg+ankle_height;// full leg length
const double hip_offset=0.07260;// from pelvis center to hip joint

const double upperarm = 0.180;
const double forearm = 0.20451;
const double shoulder_offset=0.1535;// from shoulder axis to chest center

void LegInverseKinematics(MatrixClass Waist_P,MatrixClass Waist_R,MatrixClass Foot_P,MatrixClass Foot_R,  double hipoffset, double *jointangles)
{
	const double A=upperleg; //length of thigh
	const double B=lowerleg; //length of shin

	MatrixClass Dt, R06T, r;

	Dt(1,1)=0.0;
	Dt(2,1)=hipoffset;
	Dt(3,1)=0;

	R06T=Foot_R.transpose();
	r=R06T*(Waist_P+(Waist_R*Dt)-Foot_P);//the vector points from ankle to hip with respect to local foot orientation frame
	double C;//the distance between hip and ankle joint
	double C_reset;//reset C if C is out of range
	C = sqrt(r(1,1)*r(1,1)+r(2,1)*r(2,1)+r(3,1)*r(3,1));

	double kneeMin=DEGTORAD(0); // set the minimum knee angle, avoid sigularity
	double kneeMax=DEGTORAD(120);  // set the max knee angle
	double kneeExtentionMax=sqrt(A*A+B*B-2*A*B*cos(M_PI-kneeMin));
	double kneeExtentionMin=sqrt(A*A+B*B-2*A*B*cos(M_PI-kneeMax));

	if(C>=kneeExtentionMax)
	{
		C_reset=kneeExtentionMax;
		jointangles[3]=kneeMin;
	}
	else if(C<=kneeExtentionMin)
	{
		C_reset=kneeExtentionMin;
		jointangles[3]=kneeMax;
	}
	else
	{
		C_reset=C;
		jointangles[3]= M_PI-acos((A*A+B*B-C_reset*C_reset)/(2.0*A*B));
	}

	double alpha=asin(A*sin(M_PI-jointangles[3])/C_reset);
	double rxz=sqrt(r(1,1)*r(1,1)+r(3,1)*r(3,1));
	double costheta = C_reset/sqrt(r(1,1)*r(1,1)+r(3,1)*r(3,1))*cos(M_PI/2-alpha);
	double x=rxz*sqrt(1-costheta*costheta);
	jointangles[4] =atan2(r(2,1),x);

	double y=C_reset*cos(alpha)-x;
	double gama = asin(y*sin(alpha)/rxz);
	jointangles[5] = -(atan2(r(1,1),r(3,1))+gama+alpha);

	/*-- Rfoot=Rbody*Ry(q1)*Rx(q2)*Rz(q3)*Ry(q4)*Ry(q5)*Rx(q6) ----*/
	/*-- Rfoot=Rbody*Ry(q1)*Rx(q2)*Rz(q3)*Ry(q4+q5)*Rx(q6) ----*/
	/*-- => Ry(q1)*Rx(q2)*Rz(q3)= Rbody^T * Rfoot * Rx(q6)^T * Ry(q5)^T *Ry(q4)^T --*/
	/*--or => Ry(q1)*Rx(q2)*Rz(q3)= Rbody^T * Rfoot * Rx(q6)^T * Ry(q5+q4)^T --*/
	/*Note: Rx(q)^T = Rx(-q), so we dont need to transpose, make code easier*/
	MatrixClass R34, R45, R56, Rfoot_hip, R;
	R34(1,1)=cos(jointangles[3]);
	R34(1,2)=0;
	R34(1,3)=sin(jointangles[3]);
	R34(2,1)=0;
	R34(2,2)=1;
	R34(2,3)=0;
	R34(3,1)=-sin(jointangles[3]);
	R34(3,2)=0;
	R34(3,3)=cos(jointangles[3]);
	//R34 = [cos(q4) 0 sin(q4);
	//	0 1 0;
	//	-sin(q4) 0 cos(q4)];% knee
	R45(1,1)=1;
	R45(1,2)=0;
	R45(1,3)=0;
	R45(2,1)=0;
	R45(2,2)=cos(jointangles[4]);
	R45(2,3)=-sin(jointangles[4]);
	R45(3,1)=0;
	R45(3,2)=sin(jointangles[4]);
	R45(3,3)=cos(jointangles[4]);
	//R45 = [1 0 0;
	//	0 cos(q5) -sin(q5);
	//	0 sin(q5) cos(q5)];% ankle roll

	R56(1,1)=cos(jointangles[5]);
	R56(1,2)=0;
	R56(1,3)=sin(jointangles[5]);
	R56(2,1)=0;
	R56(2,2)=1;
	R56(2,3)=0;
	R56(3,1)=-sin(jointangles[5]);
	R56(3,2)=0;
	R56(3,3)=cos(jointangles[5]);
	//R56 = [cos(q6) 0 sin(q6);
	//	0 1 0;
	//	-sin(q6) 0 cos(q6)];% ankle pitch

	Rfoot_hip=Waist_R.transpose()*Foot_R;
	R=Rfoot_hip*R56.transpose()*R45.transpose()*R34.transpose();

	jointangles[0]=atan2(R(1,3),R(3,3));
	jointangles[2]=atan2(R(2,1),R(2,2));
	jointangles[1]=asin(-R(2,3));
}


void InvKHIP(float HipP[3], float HipO[3], float LeftFootP[3], float LFtO[4], float RightFootP[3], float RFtO[4], float *traj)
{
	MatrixClass Hip_P, Hip_R, LeftFoot_Pos, LeftFoot_R, RightFoot_Pos, RightFoot_R;
	double jointanglesL[6]={0,0,0,0,0,0};
	double jointanglesR[6]={0,0,0,0,0,0};

	/*-ORIENTATION is typical Roll-Pitch-Yaw configuration (Z-Y-X Euler angles), with respect to global frame, not locale----*/
	/*body orientation, instantanous coordinate, no yaw orientation*/
	Hip_R(1,1)=cos(HipO[2])*cos(HipO[1]);
	Hip_R(1,2)=-sin(HipO[2])*cos(HipO[0])+cos(HipO[2])*sin(HipO[1])*sin(HipO[0]);
	Hip_R(1,3)=sin(HipO[2])*sin(HipO[0])+cos(HipO[2])*sin(HipO[1])*cos(HipO[0]);
	Hip_R(2,1)=sin(HipO[2])*cos(HipO[1]);
	Hip_R(2,2)=cos(HipO[2])*cos(HipO[0])+sin(HipO[2])*sin(HipO[1])*sin(HipO[0]);
	Hip_R(2,3)=-cos(HipO[2])*sin(HipO[0])+sin(HipO[2])*sin(HipO[1])*cos(HipO[0]);
	Hip_R(3,1)=-sin(HipO[1]);
	Hip_R(3,2)=cos(HipO[1])*sin(HipO[0]);
	Hip_R(3,3)=cos(HipO[1])*cos(HipO[0]);

	/*left Foot orientation*/
	LeftFoot_R(1,1)=cos(LFtO[2])*cos(LFtO[1])*cos(LFtO[3])+(-sin(LFtO[2])*cos(LFtO[0])+cos(LFtO[2])*sin(LFtO[1])*sin(LFtO[0]))*sin(LFtO[3]);
	LeftFoot_R(1,2)=-cos(LFtO[2])*cos(LFtO[1])*sin(LFtO[3])+(-sin(LFtO[2])*cos(LFtO[0])+cos(LFtO[2])*sin(LFtO[1])*sin(LFtO[0]))*cos(LFtO[3]);
	LeftFoot_R(1,3)=sin(LFtO[2])*sin(LFtO[0])+cos(LFtO[2])*sin(LFtO[1])*cos(LFtO[0]);
	LeftFoot_R(2,1)=sin(LFtO[2])*cos(LFtO[1])*cos(LFtO[3])+(cos(LFtO[2])*cos(LFtO[0])+sin(LFtO[2])*sin(LFtO[1])*sin(LFtO[0]))*sin(LFtO[3]);
	LeftFoot_R(2,2)=-sin(LFtO[2])*cos(LFtO[1])*sin(LFtO[3])+(cos(LFtO[2])*cos(LFtO[0])+sin(LFtO[2])*sin(LFtO[1])*sin(LFtO[0]))*cos(LFtO[3]);
	LeftFoot_R(2,3)=-cos(LFtO[2])*sin(LFtO[0])+sin(LFtO[2])*sin(LFtO[1])*cos(LFtO[0]);
	LeftFoot_R(3,1)=-sin(LFtO[1])*cos(LFtO[3])+cos(LFtO[1])*sin(LFtO[0])*sin(LFtO[3]);
	LeftFoot_R(3,2)=sin(LFtO[1])*sin(LFtO[3])+cos(LFtO[1])*sin(LFtO[0])*cos(LFtO[3]);
	LeftFoot_R(3,3)=cos(LFtO[1])*cos(LFtO[0]);


	/*right Foot orientation */
	RightFoot_R(1,1)=cos(RFtO[2])*cos(RFtO[1])*cos(RFtO[3])+(-sin(RFtO[2])*cos(RFtO[0])+cos(RFtO[2])*sin(RFtO[1])*sin(RFtO[0]))*sin(RFtO[3]);
	RightFoot_R(1,2)=-cos(RFtO[2])*cos(RFtO[1])*sin(RFtO[3])+(-sin(RFtO[2])*cos(RFtO[0])+cos(RFtO[2])*sin(RFtO[1])*sin(RFtO[0]))*cos(RFtO[3]);
	RightFoot_R(1,3)=sin(RFtO[2])*sin(RFtO[0])+cos(RFtO[2])*sin(RFtO[1])*cos(RFtO[0]);
	RightFoot_R(2,1)=sin(RFtO[2])*cos(RFtO[1])*cos(RFtO[3])+(cos(RFtO[2])*cos(RFtO[0])+sin(RFtO[2])*sin(RFtO[1])*sin(RFtO[0]))*sin(RFtO[3]);
	RightFoot_R(2,2)=-sin(RFtO[2])*cos(RFtO[1])*sin(RFtO[3])+(cos(RFtO[2])*cos(RFtO[0])+sin(RFtO[2])*sin(RFtO[1])*sin(RFtO[0]))*cos(RFtO[3]);
	RightFoot_R(2,3)=-cos(RFtO[2])*sin(RFtO[0])+sin(RFtO[2])*sin(RFtO[1])*cos(RFtO[0]);
	RightFoot_R(3,1)=-sin(RFtO[1])*cos(RFtO[3])+cos(RFtO[1])*sin(RFtO[0])*sin(RFtO[3]);
	RightFoot_R(3,2)=sin(RFtO[1])*sin(RFtO[3])+cos(RFtO[1])*sin(RFtO[0])*cos(RFtO[3]);
	RightFoot_R(3,3)=cos(RFtO[1])*cos(RFtO[0]);

	/*----------Position -----------*/
	/*body position*/
	Hip_P(1,1)=HipP[0];
	Hip_P(2,1)=HipP[1];
	Hip_P(3,1)=HipP[2];
	/*left foot position*/
	LeftFoot_Pos(1,1)=LeftFootP[0];
	LeftFoot_Pos(2,1)=LeftFootP[1];
	LeftFoot_Pos(3,1)=LeftFootP[2];
	/*right foot position*/
	RightFoot_Pos(1,1)=RightFootP[0];
	RightFoot_Pos(2,1)=RightFootP[1];
	RightFoot_Pos(3,1)=RightFootP[2];

	LegInverseKinematics(Hip_P,Hip_R,LeftFoot_Pos,LeftFoot_R,hip_offset, jointanglesL);
	LegInverseKinematics(Hip_P,Hip_R,RightFoot_Pos,RightFoot_R,-hip_offset, jointanglesR);

    /*Joint angles in rad for EPFL COMAN*/
	traj[3]=jointanglesR[0];//right hip pitch
	traj[4]=jointanglesL[0];//left hip pitch
	traj[5]=jointanglesR[1];//right hip roll
	traj[6]=jointanglesR[2];//right hip yaw
	traj[7]=jointanglesR[3];//right knee
	traj[8]=jointanglesR[5];//right ankle pitch
	traj[9]=jointanglesR[4];//right ankle roll
	traj[10]=jointanglesL[1];//left hip roll
	traj[11]=jointanglesL[2];//left hip yaw
	traj[12]=jointanglesL[3];//left knee
	traj[13]=jointanglesL[5];//left ankle pitch
	traj[14]=jointanglesL[4];//left ankle roll
}


void LegForwardKinematics(float q[6], float hipoffset, float*leg_pos)
{
	MatrixClass A, B, C, R01, R12, R23, R34,R45,R56, knee, ankle;
	A(1,1)=0;
	A(2,1)=0;
	A(3,1)=-upperleg;
	B(1,1)=0;
	B(2,1)=0;
	B(3,1)=-lowerleg;
	C(1,1)=0;
	C(2,1)=hipoffset;
	C(3,1)=0;

	//R01= [cos(q(1)) 0 sin(q(1));
	//	0 1 0;
	//	-sin(q(1)) 0 cos(q(1));]; // hip pitch
	R01(1,1)=cos(q[0]);
	R01(1,2)=0;
	R01(1,3)=sin(q[0]);
	R01(2,1)=0;
	R01(2,2)=1;
	R01(2,3)=0;
	R01(3,1)=-sin(q[0]);
	R01(3,2)=0;
	R01(3,3)=cos(q[0]);		// hip pitch

	//R12=[1 0 0;
	//	0 cos(q(2)) -sin(q(2));
	//	0 sin(q(2)) cos(q(2))]; // hip roll
	R12(1,1)=1;
	R12(1,2)=0;
	R12(1,3)=0;
	R12(2,1)=0;
	R12(2,2)=cos(q[1]);
	R12(2,3)=-sin(q[1]);
	R12(3,1)=0;
	R12(3,2)=sin(q[1]);
	R12(3,3)=cos(q[1]);			// hip roll

	//R23 = [cos(q(3)) -sin(q(3)) 0;
	//	sin(q(3)) cos(q(3)) 0;
	//	0 0 1];					// hip yaw
	R23(1,1)=cos(q[2]);
	R23(1,2)=-sin(q[2]);
	R23(1,3)=0;
	R23(2,1)=sin(q[2]);
	R23(2,2)=cos(q[2]);
	R23(2,3)=0;
	R23(3,1)=0;
	R23(3,2)=0;
	R23(3,3)=1;					// hip yaw

	//R34 = [cos(q(4)) 0 sin(q(4));
	//	0 1 0;
	//	-sin(q(4)) 0 cos(q(4))];// knee
	R34(1,1)=cos(q[3]);
	R34(1,2)=0;
	R34(1,3)=sin(q[3]);
	R34(2,1)=0;
	R34(2,2)=1;
	R34(2,3)=0;
	R34(3,1)=-sin(q[3]);
	R34(3,2)=0;
	R34(3,3)=cos(q[3]);	// knee

	knee=R01*R12*A+C;// no need to *R23
	ankle=knee+R01*R12*R23*R34*B; // R45 R56 are not needed to calcuate the ankle position

	leg_pos[0]=knee(1,1);
	leg_pos[1]=knee(2,1);
	leg_pos[2]=knee(3,1);
    leg_pos[3]=ankle(1,1);
	leg_pos[4]=ankle(2,1);
	leg_pos[5]=ankle(3,1);
}


/*------------------------------------------*/
void LegFWTrajectory(float q[], float legL[6], float legR[6])
{
	// q is the vector that contains all the reference/link/motor feedback from robot to pass in
	float qR[6]={q[3],q[5],q[6],q[7],q[9],q[8]};
	float qL[6]={q[4],q[10],q[11],q[12],q[14],q[13]};
	LegForwardKinematics(qL, hip_offset, legL);
	LegForwardKinematics(qR, -hip_offset,legR);
	// note that qR follows the correct order of the kinematic chain while q is the 23 element that used in the console that contains all data
    // LegL/LegR contains the xyz position of the knee and ankle respectively.
}


/*------------ below are the Fw Inv Kinematics for arms --------------- */


// Geometric solution
void ArmInverseKinematics(MatrixClass Body_P,MatrixClass Body_R,MatrixClass Hand_P,float handyaw,  double offset, double *jointangles)
{
    float q[6];
    const double a=upperarm;
	const double b=forearm;
    double C_original;
    double C_reset;//reset C if C is out of range
	MatrixClass Dt, R06T, r_original, r_scale, r, Rpitch, Rroll, Ryaw;
	Dt(1,1)=0.0;
	Dt(2,1)=offset;
	Dt(3,1)=0;
	r_original=Body_R.transpose()*(Hand_P-Body_P)-Dt;//the vector points from shoulder to wrist to shoulder with respect to local body frame
    // hand virtual orientation is not importance since it doesnt affect position
	C_original = sqrtf(r_original(1,1)*r_original(1,1)+r_original(2,1)*r_original(2,1)+r_original(3,1)*r_original(3,1));

	double elbowMin=DEGTORAD(-2);  // to avoid hitting the limit
	double elbowMax=DEGTORAD(-120);// max elbow flexion, 10 degrees margin from real limit
	double armLengthMax=sqrt(a*a+b*b-2*a*b*cos(M_PI+elbowMin));
	double armLengthMin=sqrt(a*a+b*b-2*a*b*cos(M_PI+elbowMax));

	if(C_original>=armLengthMax)
	{
		C_reset=armLengthMax;
		q[3]=elbowMin;
	}
	else if(C_original<=armLengthMin)
	{
		C_reset=armLengthMin;
		q[3]=elbowMax;
	}
	else
	{
		C_reset=C_original;
		q[3]= -(M_PI-acos((a*a+b*b-C_reset*C_reset)/(2.0*a*b)));
	}
    r_scale(1,1)=r_original(1,1)*C_reset/C_original;
	r_scale(2,1)=r_original(2,1)*C_reset/C_original;
	r_scale(3,1)=r_original(3,1)*C_reset/C_original;

	float pitch, roll, rxz;
	rxz=sqrtf(r_scale(1,1)*r_scale(1,1)+r_scale(3,1)*r_scale(3,1));
	pitch=atan2(-r_scale(1,1),-r_scale(3,1));
	roll=atan2(r_scale(2,1),rxz);

    Rpitch(1,1)=cos(pitch);
	Rpitch(1,2)=0;
	Rpitch(1,3)=sin(pitch);
	Rpitch(2,1)=0;
	Rpitch(2,2)=1;
	Rpitch(2,3)=0;
	Rpitch(3,1)=-sin(pitch);
	Rpitch(3,2)=0;
	Rpitch(3,3)=cos(pitch);    // hand virtual pitch

    Rroll(1,1)=1;
	Rroll(1,2)=0;
	Rroll(1,3)=0;
	Rroll(2,1)=0;
	Rroll(2,2)=cos(roll);
	Rroll(2,3)=-sin(roll);
	Rroll(3,1)=0;
	Rroll(3,2)=sin(roll);
	Rroll(3,3)=cos(roll);	// hand virtual roll

    Ryaw(1,1)=cos(handyaw);
	Ryaw(1,2)=-sin(handyaw);
	Ryaw(1,3)=0;
	Ryaw(2,1)=sin(handyaw);
	Ryaw(2,2)=cos(handyaw);
	Ryaw(2,3)=0;
	Ryaw(3,1)=0;
	Ryaw(3,2)=0;
	Ryaw(3,3)=1;	// hand virtual yaw specified by user

    MatrixClass Hand_R;
    Hand_R=Rpitch*Rroll*Ryaw; // here the yaw is the local yaw around the vector from shoulder to hand

	R06T=Hand_R.transpose();
	r=R06T*(Body_P+(Body_R*Dt)-Hand_P);//the vector points from wrist to shoulder with respect to local wrist orientation frame

	double alpha=asin(a*sin(M_PI+q[3])/C_reset);//the angle at the bottom in triangle
	q[5] = atan2(r(2,1),r(3,1));//ankle roll -pi/2<q6<pi/2
	if (q[5]>M_PI/2.0)
	{
		q[5] = q[5]-M_PI;
	}
	else if (q[5]<-M_PI/2.0)
	{
		q[5]= q[5]+M_PI;
	}

	// ankle pitch
	if (r(3,1)>0.0)
	{
		q[4] = -( atan2( r(1,1),sqrt(r(2,1)*r(2,1)+r(3,1)*r(3,1)) ) - alpha );
	}
	else
	{
		q[4] = -( atan2( r(1,1),-sqrt(r(2,1)*r(2,1)+r(3,1)*r(3,1)) ) - alpha );
	}

	/*-- Rfoot=Rbody*Ry(q1)*Rx(q2)*Rz(q3)*Ry(q4)*Ry(q5)*Rx(q6) ----*/
	/*-- Rfoot=Rbody*Ry(q1)*Rx(q2)*Rz(q3)*Ry(q4+q5)*Rx(q6) ----*/
	/*-- => Ry(q1)*Rx(q2)*Rz(q3)= Rbody^T * Rfoot * Rx(q6)^T * Ry(q5)^T *Ry(q4)^T --*/
	/*--or => Ry(q1)*Rx(q2)*Rz(q3)= Rbody^T * Rfoot * Rx(q6)^T * Ry(q5+q4)^T --*/
	/*Note: Rx(q)^T = Rx(-q), so we dont need to transpose, make code easier*/
	MatrixClass Rxq6,Ryq45,R,RbodyT;

	RbodyT = Body_R.transpose();
	Rxq6(1,1)=1;
	Rxq6(1,2)=0;
	Rxq6(1,3)=0;
    Rxq6(2,1)=0;
	Rxq6(2,2)=cos(-q[5]);
	Rxq6(2,3)=-sin(-q[5]);
    Rxq6(3,1)=0;
	Rxq6(3,2)=sin(-q[5]);
	Rxq6(3,3)=cos(-q[5]);

	Ryq45(1,1)=cos(-q[3]-q[4]);
	Ryq45(1,2)=0;
	Ryq45(1,3)=sin(-q[3]-q[4]);
    Ryq45(2,1)=0;
	Ryq45(2,2)=1;
	Ryq45(2,3)=0;
    Ryq45(3,1)=-sin(-q[3]-q[4]);
	Ryq45(3,2)=0;
	Ryq45(3,3)=cos(-q[3]-q[4]);

	R = RbodyT*( Hand_R*(Rxq6*Ryq45) );
	/*Ry(q1)*Rx(q2)*Rz(q3)=R*/
	//here, the pitch-roll-yaw means the joint configuration of the arm/shoulder.
/*
	Ry*Rx*Rz=
	[  cos(yaw)*cos(pitch)+sin(yaw)*sin(roll)*sin(pitch), -sin(yaw)*cos(pitch)+cos(yaw)*sin(roll)*sin(pitch),                               cos(roll)*sin(pitch)]
	[                                 sin(yaw)*cos(roll),                                 cos(yaw)*cos(roll),                                         -sin(roll)]
	[ -cos(yaw)*sin(pitch)+sin(yaw)*sin(roll)*cos(pitch),  sin(yaw)*sin(pitch)+cos(yaw)*sin(roll)*cos(pitch),                               cos(roll)*cos(pitch)]
*/
 	q[2] = atan2(R(2,1),R(2,2)); //Z hip yaw  Y-X-Z
	q[0] = atan2(R(1,3),R(3,3)); //Y hip pitch
	//q[1] = atan2(-R(2,3),(R(1,3)*sin(jointangles[0]) + R(3,3)*cos(jointangles[0]))); // it is the same as asin(-R(2,3))
	q[1]=asin(-R(2,3)); // the same
	for(int i=0;i<4;i++)
	{
	    jointangles[i]=q[i]; // asign value
	}
}


void ArmIKTrajectory(float BodyP[3], float BodyO[3],float LeftHandP[3],float yawArmL, float RightHandP[3],float yawArmR,float *traj)
{
	MatrixClass Body_P, Body_R, LeftHand_Pos, LeftHand_R, RightHand_Pos, RightHand_R;
	double leftArm[4];// last tow joints (wrist) dont exist
	double rightArm[4];

	/*body orientation*/
	Body_R(1,1)=cos(BodyO[2])*cos(BodyO[1]);
	Body_R(1,2)=-sin(BodyO[2])*cos(BodyO[0])+cos(BodyO[2])*sin(BodyO[1])*sin(BodyO[0]);
	Body_R(1,3)=sin(BodyO[2])*sin(BodyO[0])+cos(BodyO[2])*sin(BodyO[1])*cos(BodyO[0]);
	Body_R(2,1)=sin(BodyO[2])*cos(BodyO[1]);
	Body_R(2,2)=cos(BodyO[2])*cos(BodyO[0])+sin(BodyO[2])*sin(BodyO[1])*sin(BodyO[0]);
	Body_R(2,3)=-cos(BodyO[2])*sin(BodyO[0])+sin(BodyO[2])*sin(BodyO[1])*cos(BodyO[0]);
	Body_R(3,1)=-sin(BodyO[1]);
	Body_R(3,2)=cos(BodyO[1])*sin(BodyO[0]);
	Body_R(3,3)=cos(BodyO[1])*cos(BodyO[0]);

	/*body position*/
	Body_P(1,1)=BodyP[0];
	Body_P(2,1)=BodyP[1];
	Body_P(3,1)=BodyP[2];
	/*left foot position*/
	LeftHand_Pos(1,1)=LeftHandP[0];
	LeftHand_Pos(2,1)=LeftHandP[1];
	LeftHand_Pos(3,1)=LeftHandP[2];
	/*right foot position*/
	RightHand_Pos(1,1)=RightHandP[0];
	RightHand_Pos(2,1)=RightHandP[1];
	RightHand_Pos(3,1)=RightHandP[2];

    ArmInverseKinematics(Body_P,Body_R,LeftHand_Pos, yawArmL, shoulder_offset,leftArm);
	ArmInverseKinematics(Body_P,Body_R,RightHand_Pos,yawArmR, -shoulder_offset,rightArm);
	/*Joint angles of arms */
	traj[15]=rightArm[0];// right arm pitch
	traj[16]=rightArm[1]+0.5*M_PI;// right arm roll
	traj[17]=rightArm[2];// right arm yaw
	traj[18]=rightArm[3];// right elbow

	traj[19]=leftArm[0];// left arm pitch
	traj[20]=leftArm[1]-0.5*M_PI;// left arm roll
	traj[21]=leftArm[2];// left arm yaw
	traj[22]=leftArm[3];// left elbow
}

/*------------------------------------------*/
void ArmForwardKinematics(float q[], float shoulderoffset, float*arm_pos)
{
	MatrixClass A, B, C, R01, R12, R23, R34, elbow, wrist;
	A(1,1)=0;
	A(2,1)=0;
	A(3,1)=-upperarm;
	B(1,1)=0;
	B(2,1)=0;
	B(3,1)=-forearm;
	C(1,1)=0;
	C(2,1)=shoulderoffset;
	C(3,1)=0;

	//R01= [cos(q(1)) 0 sin(q(1));
	//	0 1 0;
	//	-sin(q(1)) 0 cos(q(1));]; // shoulder pitch
	R01(1,1)=cos(q[0]);
	R01(1,2)=0;
	R01(1,3)=sin(q[0]);
	R01(2,1)=0;
	R01(2,2)=1;
	R01(2,3)=0;
	R01(3,1)=-sin(q[0]);
	R01(3,2)=0;
	R01(3,3)=cos(q[0]);		// shoulder pitch

	//R12=[1 0 0;
	//	0 cos(q(2)) -sin(q(2));
	//	0 sin(q(2)) cos(q(2))]; // shoulder roll
	R12(1,1)=1;
	R12(1,2)=0;
	R12(1,3)=0;
	R12(2,1)=0;
	R12(2,2)=cos(q[1]);
	R12(2,3)=-sin(q[1]);
	R12(3,1)=0;
	R12(3,2)=sin(q[1]);
	R12(3,3)=cos(q[1]);			// shoulder roll

	//R23 = [cos(q(3)) -sin(q(3)) 0;
	//	sin(q(3)) cos(q(3)) 0;
	//	0 0 1];					// shoulder yaw
	R23(1,1)=cos(q[2]);
	R23(1,2)=-sin(q[2]);
	R23(1,3)=0;
	R23(2,1)=sin(q[2]);
	R23(2,2)=cos(q[2]);
	R23(2,3)=0;
	R23(3,1)=0;
	R23(3,2)=0;
	R23(3,3)=1;					// shoulder yaw

	//R34 = [cos(q(4)) 0 sin(q(4));
	//	0 1 0;
	//	-sin(q(4)) 0 cos(q(4))];// elbow
	R34(1,1)=cos(q[3]);
	R34(1,2)=0;
	R34(1,3)=sin(q[3]);
	R34(2,1)=0;
	R34(2,2)=1;
	R34(2,3)=0;
	R34(3,1)=-sin(q[3]);
	R34(3,2)=0;
	R34(3,3)=cos(q[3]);			// elbow

	elbow=R01*R12*R23*A+C;
	wrist=elbow+R01*R12*R23*R34*B;

	arm_pos[0]=elbow(1,1);
	arm_pos[1]=elbow(2,1);
	arm_pos[2]=elbow(3,1);
	arm_pos[3]=wrist(1,1);
	arm_pos[4]=wrist(2,1);
	arm_pos[5]=wrist(3,1);
}
/*------------------------------------------*/
void ArmFWTrajectory(float q[], float armL[6], float armR[6])
{
	// q is the vector that contains all the link/motor feedback from robot
	float qL[4]={q[19],q[20]+0.5*M_PI,q[21],q[22]};
	float qR[4]={q[15],q[16]-0.5*M_PI,q[17],q[18]};
	ArmForwardKinematics(qL, shoulder_offset, armL);
	ArmForwardKinematics(qR, -shoulder_offset, armR);
}
/*------------------------------------------*/
