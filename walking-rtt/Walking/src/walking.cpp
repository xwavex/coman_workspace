#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <fstream>

//Class headers
#include "walking.hpp"
#include "init_pos.hh"
#include "imu_data.hh"
#include "Control.hh"
// #include "R2Euler.hh"

//orocos-rtt headers
#include <rtt/RTT.hpp>
#include <rtt/Component.hpp>
#include <orocos/rtt_roscomm/rtt_rostopic.h>
#include <orocos/rtt_rosclock/rtt_rosclock.h>

//Eigen headers
#include </usr/include/eigen3/Eigen/Dense>

// needed for the macro at the end of this file:
using namespace std;
using namespace RTT;
using namespace Eigen;

double vals[NUM];
static double begin_time = -1;
double _time;
static double Q0[NUM];
const double TIME1 = 1, TIME2 = 2;
// const double DISABLED_MOTORS[3] = {29, 39, 39};
double  qSens[NUM], dqSens[NUM], tauSens[NUM], qSensAbs[NUM];
double trans[3][3];
double forceRightAnkle[3], torqueRightAnkle[3], forceLeftAnkle[3], torqueLeftAnkle[3], forceRightHand[3], forceLeftHand[3];

const double TORQUE_CORR = -1;  // Torque correction from real to webots
double tauDes[NUM];
double forceSensors[3]={0};
double valsLowerBody[LOWER_BODY_N];
double valsUpperBody [NUM];
static ifstream inputfile, inputfile0;
static ofstream outputfile, opFileInit;
double ignoreVar;
double h[NUM], dh[NUM], hD[NUM], dhD[NUM];
Control control;
double T0=0;
static double qknee0 = 0.2, qP0 = -0.1, qR0 = 0.055*1;
const double RIGHT_ELBOW_YAW = 0, LEFT_ELBOW_YAW = 0;

// Note: Q_INIT[NUM] in the Control.cc is the same as qInit
// static double qInit[NUM] = {0,0.075,0,qP0,qP0,-qR0,0,qknee0,qP0*1.4,qR0,qR0*1,0,qknee0,qP0*1.4,-qR0*1, 0.45, -0.2 , 0.0, -1.75, 0.45, 0.2, 0.0, -1.75, RIGHT_ELBOW_YAW,0.0,0.0,LEFT_ELBOW_YAW,0,0,0,0};
static double qInit[NUM] = {0,0.075,0,qP0,qP0,-qR0,0, qknee0,qP0*1.4,qR0,qR0*1,0,qknee0,qP0*1.4,-qR0*1, 0.45, -0.2 , 0.0, -1.75, 0.45, 0.2, 0.0, -1.75, RIGHT_ELBOW_YAW,0.0,0.0,LEFT_ELBOW_YAW,0,0,0,0};

#ifndef REAL_ROBOT
    const double TIME2WALK=10;
#else
    const double TIME2WALK=10;
#endif

// unsigned int whichComan_ = 1;

/*
    @brief: Walking constructor, can be used to add properties/operations
            other than the 5 main hooks.
        REAL_ROBOT = FALSE -> Simulator 
*/
Walking::Walking(string const &name) : TaskContext(name) 
{
    //  this->addProperty("use_real_robot", use_real_robot);
    #ifdef REAL_ROBOT
        this->addOperation("loadURDFAndSRDF", &Walking::loadURDFAndSRDF, this, RTT::ClientThread);

        this->addOperation("attachToRobot",&Walking::attachToRobot, this, ClientThread);
    #else
        //ROS IMU Sensor port
        _imu_port.setName("imu_port");
    
    #endif
}

/*
    @brief: Intializations and object creations go here. 
                Each component should run this before being 
                able to run check for port connectivity
            ->Added ROS bridge
*/
bool Walking::configureHook() 
{   
    left_arm_joint_data_store = rstrt::robot::JointState(COMAN_LEFT_ARM_DOF_SIZE);
    left_arm_joint_data.setName("left_arm_joint_data");

    right_arm_joint_data_store = rstrt::robot::JointState(COMAN_RIGHT_ARM_DOF_SIZE);
    right_arm_joint_data.setName("right_arm_joint_data");

    left_leg_joint_data_store = rstrt::robot::JointState(COMAN_LEFT_LEG_DOF_SIZE);
    left_leg_joint_data.setName("left_leg_joint_data");

    right_leg_joint_data_store = rstrt::robot::JointState(COMAN_RIGHT_LEG_DOF_SIZE);
    right_leg_joint_data.setName("right_leg_joint_data");

    torso_joint_data_store = rstrt::robot::JointState(COMAN_TORSO_DOF_SIZE);
    torso_joint_data.setName("torso_joint_data");

    right_foot_wrench_store = rstrt::dynamics::Wrench();
    right_foot_wrench.setName("right_foot_wrench");

    left_foot_wrench_store = rstrt::dynamics::Wrench();
    left_foot_wrench.setName("left_foot_wrench");

    right_arm_wrench_store = rstrt::dynamics::Wrench();
    right_arm_wrench.setName("right_arm_wrench");

    left_arm_wrench_store = rstrt::dynamics::Wrench();
    left_arm_wrench.setName("left_arm_wrench");

#ifdef REAL_ROBOT
    outputfile.open("/home/biorob/coman_workspace/realData/tempdataRealTime.txt");
    opFileInit.open("/home/biorob/coman_workspace/realData/tempdata_initRealtime.txt");

    right_arm_voltage_command = rstrt::kinematics::JointAngles(COMAN_RIGHT_ARM_DOF_SIZE);
    right_arm_voltage_command.angles.setZero(7);
    right_arm_voltage.setName("right_arm_voltage");
    right_arm_voltage.setDataSample(right_arm_voltage_command);

    left_arm_voltage_command = rstrt::kinematics::JointAngles(COMAN_LEFT_ARM_DOF_SIZE);
    left_arm_voltage_command.angles.setZero(7);
    left_arm_voltage.setName("left_arm_voltage");
    left_arm_voltage.setDataSample(left_arm_voltage_command);

    right_leg_voltage_command = rstrt::kinematics::JointAngles(COMAN_RIGHT_LEG_DOF_SIZE);
    right_leg_voltage_command.angles.setZero(6);
    right_leg_voltage.setName("right_leg_voltage");
    right_leg_voltage.setDataSample(right_leg_voltage_command);

    left_leg_voltage_command = rstrt::kinematics::JointAngles(COMAN_LEFT_LEG_DOF_SIZE);
    left_leg_voltage_command.angles.setZero(6);
    left_leg_voltage.setName("left_leg_voltage");
    left_leg_voltage.setDataSample(left_leg_voltage_command);

    torso_voltage_command =rstrt::kinematics::JointAngles(COMAN_TORSO_DOF_SIZE);
    torso_voltage_command.angles.setZero(3);
    torso_voltage.setName("torso_voltage");
    torso_voltage.setDataSample(torso_voltage_command);

    ports()->addPort(right_arm_voltage);
    ports()->addPort(left_arm_voltage);
    ports()->addPort(right_leg_voltage);
    ports()->addPort(left_leg_voltage);
    ports()->addPort(torso_voltage);

    imu_port.setName("imu_port");
    // imu_data_.setZero(3);
    ports()->addPort(imu_port);
#else
    outputfile.open("/home/biorob/coman_workspace/simData/tempdataSimTime.txt");
    opFileInit.open("/home/biorob/coman_workspace/simData/tempdata_initSimTime.txt");

    right_arm_torque_command = rstrt::dynamics::JointTorques(COMAN_RIGHT_ARM_DOF_SIZE);
    right_arm_torque_command.torques.setZero(COMAN_RIGHT_ARM_DOF_SIZE);
    right_arm_torque.setName("right_arm_torque");
    right_arm_torque.setDataSample(right_arm_torque_command);

    left_arm_torque_command = rstrt::dynamics::JointTorques(COMAN_LEFT_ARM_DOF_SIZE);
    left_arm_torque_command.torques.setZero(COMAN_LEFT_ARM_DOF_SIZE);
    left_arm_torque.setName("left_arm_torque");
    left_arm_torque.setDataSample(left_arm_torque_command);

    right_leg_torque_command = rstrt::dynamics::JointTorques(COMAN_RIGHT_LEG_DOF_SIZE);
    right_leg_torque_command.torques.setZero(COMAN_RIGHT_LEG_DOF_SIZE);
    right_leg_torque.setName("right_leg_torque");
    right_leg_torque.setDataSample(right_leg_torque_command);

    left_leg_torque_command = rstrt::dynamics::JointTorques(COMAN_LEFT_LEG_DOF_SIZE);
    left_leg_torque_command.torques.setZero(COMAN_LEFT_LEG_DOF_SIZE);
    left_leg_torque.setName("left_leg_torque");
    left_leg_torque.setDataSample(left_leg_torque_command);

    torso_torque_command = rstrt::dynamics::JointTorques(COMAN_TORSO_DOF_SIZE);
    torso_torque_command.torques.setZero(COMAN_TORSO_DOF_SIZE);
    torso_torque.setName("torso_torque");
    torso_torque.setDataSample(torso_torque_command);

    ports()->addPort(left_arm_torque);
    ports()->addPort(right_arm_torque);
    ports()->addPort(left_leg_torque);
    ports()->addPort(right_leg_torque);
    ports()->addPort(torso_torque);

    //ROS
    ports()->addPort(_imu_port).doc("Joint State desired from ROS gui");
    // ports()->addPort(_topic_port).doc("Rosclock time");

#endif

    ports()->addPort(left_arm_joint_data);
    ports()->addPort(right_arm_joint_data);
    ports()->addPort(left_leg_joint_data);
    ports()->addPort(right_leg_joint_data);
    ports()->addPort(torso_joint_data);
    ports()->addPort(right_foot_wrench);
    ports()->addPort(left_foot_wrench);
    ports()->addPort(right_arm_wrench);
    ports()->addPort(left_arm_wrench);

    cout<<"/********Ports Added*********/\n";

    whichComan_ = 1;

    return true;
}

/*
    @brief: Method to start all components.
*/
bool Walking::startHook() 
{
    _time = getSimulationTime();

    #ifndef REAL_ROBOT
        //Initialize rostopic imu stream
        _imu_port.createStream(rtt_roscomm::topic("/imu"));
        //Initialize rostopic clock stream
        // _topic_port_.createStream(rtt_roscomm::topic("/rosout"));
        
    #else

        task_ptr = this->getPeer(_robot_name);
        
        if(!task_ptr)
        {
            RTT::log(RTT::Error)<<"Can not getPeer("<<_robot_name<<")"<<RTT::endlog();

            return false;
        }

        if(task_ptr)
        {
            task_ptr = this->getPeer(_robot_name);
        
            if(!task_ptr)
            {
                RTT::log(RTT::Error)<<"Can not getPeer("<<_robot_name<<")"<<RTT::endlog();

                return false;
            }

            RTT::OperationCaller<std::map<std::string, std::vector<std::string> >(void) > getKinematicChainsAndJoints = task_ptr->getOperation("getKinematicChainsAndJoints");

            map<string, vector<string>> _map_kin_chains_joints = getKinematicChainsAndJoints();

            bool a = true;

            RTT::OperationCaller<bool(const std::string&, const std::vector<int>&, const std::vector<int>&, const std::vector<int>&) > setPID = task_ptr->getOperation("setPID");

            std::map<std::string, std::vector<std::string>>::iterator it;
            for(it = _map_kin_chains_joints.begin(); it != _map_kin_chains_joints.end(); it++)
            {
                

                std::cout<<"Here";
                std::vector<int> P, I, D;
                for(unsigned int i = 0; i < it->first.size(); i++)
                {
                    P.push_back(0);
                    I.push_back(0);
                    D.push_back(0);
                }
                a = setPID(it->first, P, I, D);
                std::cout<<"Exit with : " << a << std::endl;
            }

            if(a)
            {
                right_arm_voltage_command.angles.setZero(7);
                left_arm_voltage_command.angles.setZero(7);
                right_leg_voltage_command.angles.setZero(6);
                left_leg_voltage_command.angles.setZero(6);
                torso_voltage_command.angles.setZero(3);

                right_arm_voltage.write(right_arm_voltage_command);
                left_arm_voltage.write(left_arm_voltage_command);
                right_leg_voltage.write(right_leg_voltage_command);
                left_leg_voltage.write(left_leg_voltage_command);
                torso_voltage.write(torso_voltage_command);
                
                return true;
            }
            else
            {
                log(Warning)<<"Something went wrong during setPID!"<<endlog();
            }
        }
        else
            log(Warning)<<"task pointer is null, call attach to robot first!"<<endlog();

    #endif

    return true;
}

/*
    @brief: Loop to run until the end of simulation.
            (Remember to set end to simulation)
*/
void Walking::updateHook() 
{
     _time = getSimulationTime();

    #ifndef REAL_ROBOT
    //IMU sensor read angles as Quaternion
    geometry_msgs::Quaternion Q_imu;
    FlowStatus fs = _imu_port.read(_imu_msg);
    // FlowStatus fsClk = _topic_port_.read(_topic_msg);
    double Trans[3][3];
    double ImuAngRates[3];
    double ImuAccelerations[3];
    imu_data IMU_data;
 
    Q_imu.x=_imu_msg.orientation.x;
    Q_imu.y=_imu_msg.orientation.y;
    Q_imu.z=_imu_msg.orientation.z;
    Q_imu.w=_imu_msg.orientation.w;
/*
    //Debug to check if port is working
    cout<<"Magnitude should be 1 = "<<(Q_imu.x*Q_imu.x)+(Q_imu.y*Q_imu.y)+(Q_imu.z*Q_imu.z)+(Q_imu.w*Q_imu.w)<<endl;
    cout<<"y="<<Q_imu.y<<endl;
    cout<<"z="<<Q_imu.z<<endl;
    cout<<"w="<<Q_imu.w<<endl;
*/
/*
    Sensor Orientation Matrix is as follows:
    [0 3 6
     1 4 7
     2 5 8]
*/
    // Trans[0][0] = 2*((Q_imu.x*Q_imu.x) + (Q_imu.w*Q_imu.w)) - 1;
    // Trans[0][1] = 2*((Q_imu.x*Q_imu.y) + (Q_imu.w*Q_imu.z));
    // Trans[0][2] = 2*((Q_imu.x*Q_imu.z) - (Q_imu.w*Q_imu.y));
    // Trans[1][0] = 2*((Q_imu.x*Q_imu.y) - (Q_imu.w*Q_imu.z));
    // Trans[1][1] = 2*((Q_imu.w*Q_imu.w) + (Q_imu.y*Q_imu.y)) - 1;
    // Trans[1][2] = 2*((Q_imu.y*Q_imu.z) + (Q_imu.w*Q_imu.x));
    // Trans[2][0] = 2*((Q_imu.x*Q_imu.z) + (Q_imu.w*Q_imu.y));
    // Trans[2][1] = 2*((Q_imu.y*Q_imu.z) - (Q_imu.w*Q_imu.x));
    // Trans[2][2] = 2*((Q_imu.w*Q_imu.w) + (Q_imu.z*Q_imu.z)) - 1;

    Trans[0][0] = 2*((Q_imu.x*Q_imu.x) + (Q_imu.w*Q_imu.w)) - 1;
    Trans[0][1] = 2*((Q_imu.x*Q_imu.y) - (Q_imu.w*Q_imu.z));
    Trans[0][2] = 2*((Q_imu.x*Q_imu.z) + (Q_imu.w*Q_imu.y));
    Trans[1][0] = 2*((Q_imu.x*Q_imu.y) + (Q_imu.w*Q_imu.z));
    Trans[1][1] = 2*((Q_imu.w*Q_imu.w) + (Q_imu.y*Q_imu.y)) - 1;
    Trans[1][2] = 2*((Q_imu.y*Q_imu.z) - (Q_imu.w*Q_imu.x));
    Trans[2][0] = 2*((Q_imu.x*Q_imu.z) - (Q_imu.w*Q_imu.y));
    Trans[2][1] = 2*((Q_imu.y*Q_imu.z) + (Q_imu.w*Q_imu.x));
    Trans[2][2] = 2*((Q_imu.w*Q_imu.w) + (Q_imu.z*Q_imu.z)) - 1;

    ImuAngRates[0] =_imu_msg.angular_velocity.x;
    ImuAngRates[1] =_imu_msg.angular_velocity.y;
    ImuAngRates[2] =_imu_msg.angular_velocity.z;

    ImuAccelerations[0] = _imu_msg.linear_acceleration.x;
    ImuAccelerations[1] = _imu_msg.linear_acceleration.y;
    ImuAccelerations[2] = _imu_msg.linear_acceleration.z;

    // double imuOrientation[3];
    // Eigen::Vector3f E_imu;

    // R2Euler(Trans, imuOrientation);
    // double thr = imuOrientation[0]; // 0
    // double thp = imuOrientation[1]; // 1
    // double thy = imuOrientation[2];

    // E_imu[0] = thp;
    // E_imu[1] = thr;
    // E_imu[2] = thy;

    // cout << E_imu[0] << " : " << E_imu[1] << " : ";

    // double testTrans[3][3];
    // toTrans(E_imu, testTrans);

    #else
    // std::cout<<"time : "<<_time<<std::endl;

    FlowStatus fs = imu_port.read(imu_data_);
    
    Eigen::Vector3f E_imu;
    double Trans[3][3];
    double ImuAngRates[3];
    double ImuAccelerations[3];
    
    // cout << imu_data_ << endl;
    
    E_imu[0]= (imu_data_[0] + (M_PI/2));   // thp
    E_imu[1]= (imu_data_[1]);              // thr
    E_imu[2]= -(imu_data_[2]);              // thy

    // cout << E_imu[0] << " : " << E_imu[1] << " thr: " << E_imu[2] << " : ";

    toTrans(E_imu, Trans);

    // double imuRot [3][3];

    // ImuAngRates[0] =imu_data_(4);   //typecast to double from float?
    // ImuAngRates[1] =imu_data_(5);
    // ImuAngRates[2] =imu_data_(6);

    // ImuAccelerations[0] = imu_data_(7);
    // ImuAccelerations[1] = imu_data_(8);
    // ImuAccelerations[2] = imu_data_(9);

    #endif

   
    //Robot interface sensory input START
    left_arm_joint_data.read(left_arm_joint_data_store);
    right_arm_joint_data.read(right_arm_joint_data_store);
    left_leg_joint_data.read(left_leg_joint_data_store);
    right_leg_joint_data.read(right_leg_joint_data_store);
    torso_joint_data.read(torso_joint_data_store);
    right_foot_wrench.read(right_foot_wrench_store);
    left_foot_wrench.read(left_foot_wrench_store);
    right_arm_wrench.read(right_arm_wrench_store);
    left_arm_wrench.read(left_arm_wrench_store);

    qSens[0] = torso_joint_data_store.angles(2);
    dqSens[0] = torso_joint_data_store.velocities(2);

    qSens[1] = torso_joint_data_store.angles(1);
    dqSens[1] = torso_joint_data_store.velocities(1);

    qSens[2] = torso_joint_data_store.angles(0);
    dqSens[2] = torso_joint_data_store.velocities(0);


    qSens[3] = right_leg_joint_data_store.angles(0);
    dqSens[3] = right_leg_joint_data_store.velocities(0);

    qSens[4] = left_leg_joint_data_store.angles(0);
    dqSens[4] = left_leg_joint_data_store.velocities(0);

    for (int i = 0; i < 3; i++)
    {
        qSens[5 + i] = right_leg_joint_data_store.angles(i + 1);
        dqSens[5 + i] = right_leg_joint_data_store.velocities(i + 1);
        qSens[10 + i] = left_leg_joint_data_store.angles(i + 1);
        dqSens[10 + i] = left_leg_joint_data_store.velocities(i + 1);
    }

    qSens[8] = right_leg_joint_data_store.angles(5);
    dqSens[8] = right_leg_joint_data_store.velocities(5);
    qSens[13] = left_leg_joint_data_store.angles(5);
    dqSens[13] = left_leg_joint_data_store.velocities(5);

    qSens[9] = right_leg_joint_data_store.angles(4);
    dqSens[9] = right_leg_joint_data_store.velocities(4);
    qSens[14] = left_leg_joint_data_store.angles(4);
    dqSens[14] = left_leg_joint_data_store.velocities(4);

    for (int i = 0; i < 4; i++)
    {
        qSens[15 + i] = right_arm_joint_data_store.angles(i);
        qSens[19 + i] = left_arm_joint_data_store.angles(i);
        dqSens[15 + i] = right_arm_joint_data_store.velocities(i);
        dqSens[19 + i] = left_arm_joint_data_store.velocities(i);
    }

    for (int i = 0; i < 3; i++)
    {
        qSens[23 + i] = right_arm_joint_data_store.angles(4 + i);
        qSens[26 + i] = left_arm_joint_data_store.angles(4 + i);
        dqSens[23 + i] = right_arm_joint_data_store.velocities(4 + i);
        dqSens[26 + i] = left_arm_joint_data_store.velocities(4 + i);
    }

    for (int i = 0; i < 3; i++)
    {
        forceRightAnkle[i] = right_foot_wrench_store.forces(i);
        forceLeftAnkle[i] = left_foot_wrench_store.forces(i);
        forceRightHand[i] = right_arm_wrench_store.forces(i);
        forceLeftHand[i] = left_arm_wrench_store.forces(i);
        torqueRightAnkle[i] = right_foot_wrench_store.torques(i);
        torqueLeftAnkle[i] = left_foot_wrench_store.torques(i);
        // torqueRightHand[i] = right_arm_wrench_store.torques(i);
        // torqueLeftHand[i] = left_arm_wrench_store.torques(i);
    }

    if (begin_time == -1)
    {
        begin_time = _time;
        for (int i = 0; i < NUM; i++)
        {
            Q0[i] = qSens[i];
        }
    }

    _time -= begin_time;

    //Robot interface sensory input END
    if (_time < TIME2WALK)
    {
        init_pos(_time, Q0, qInit, qSens, dqSens, tauDes, vals, whichComan_);
        // cout << qSens[8] << " : "<<  qSens[9] << endl;
        opFileInit << _time;
        for (int i = 0; i < 3; i++)
        {
            opFileInit << " " << forceRightAnkle[i] ;
        }
        // start_id = 3
        for (int i = 0; i < 3; i++)
        {
            opFileInit << " " << forceLeftAnkle[i] ;
        }
        opFileInit << endl;
    }
    else
    {
    //controller(outputFile, _time, Q0, qSens, qSensAbs, dqSens, tauSens, forceRightAnkle,
    //            forceLeftAnkle, torqueRightAnkle, torqueLeftAnkle, forceRightHand, 
    //            forceLeftHand, Trans, ImuAngRates, imuAccelerations, h, dh, hD, dhD, tauDes, vals);
          
        control.LowerBody(_time, Q0, qSens, qSensAbs, dqSens, tauSens, forceRightAnkle,
                            forceLeftAnkle, torqueRightAnkle, torqueLeftAnkle, forceRightHand, forceLeftHand, 
                            Trans, ImuAngRates, ImuAccelerations, h, dh, hD, dhD, tauDes, vals);
        
        control.SaveVars(outputfile);
    }
   
    //    cout << "Sim Time : " << _time << endl;

#ifdef REAL_ROBOT
    torso_voltage_command.angles(0) = vals[2];
    torso_voltage_command.angles(1) = vals[1];
    torso_voltage_command.angles(2) = vals[0];

    right_leg_voltage_command.angles(0)= vals[3];
    left_leg_voltage_command.angles(0)= vals[4];

    for (int i = 0; i < 3; i++)
    {
       right_leg_voltage_command.angles(i + 1)= vals[5 + i];
       left_leg_voltage_command.angles(i + 1)= vals[10 + i];
    }

    right_leg_voltage_command.angles(4)= vals[9];
    left_leg_voltage_command.angles(4)= vals[14];

    right_leg_voltage_command.angles(5)= vals[8];
    left_leg_voltage_command.angles(5)= vals[13];

    for (int i = 0; i < 4; i++)
    {
        right_arm_voltage_command.angles(i)= vals[15 + i];
        left_arm_voltage_command.angles(i)= vals[19 + i];
    }

    for (int i = 0; i < 3; i++)
    {
        right_arm_voltage_command.angles(4 + i)= vals[23 + i];
        left_arm_voltage_command.angles(4 + i)= vals[26 + i];
    }

    right_arm_voltage.write(right_arm_voltage_command);
    left_arm_voltage.write(left_arm_voltage_command);
    right_leg_voltage.write(0.0);
    left_leg_voltage.write(left_leg_voltage_command);
    torso_voltage.write(torso_voltage_command);

    // cout<<right_leg_voltage_command<<endl;
#endif

#ifndef REAL_ROBOT ///TODO: CHANGE SIGNS ORDER IN ANKLES!
    torso_torque_command.torques(0) = tauDes[2];
    torso_torque_command.torques(1) = tauDes[1];
    torso_torque_command.torques(2) = tauDes[0];

    right_leg_torque_command.torques(0) = tauDes[3];
    left_leg_torque_command.torques(0) = tauDes[4];

    for (int i = 0; i < 3; i++)
    {
        right_leg_torque_command.torques(i + 1) = tauDes[5 + i];
        left_leg_torque_command.torques(i + 1) = tauDes[10 + i];
    }

    right_leg_torque_command.torques(4)= tauDes[9];
    left_leg_torque_command.torques(4)= tauDes[14];

    right_leg_torque_command.torques(5)= tauDes[8];
    left_leg_torque_command.torques(5)= tauDes[13];

    for (int i = 0; i < 4; i++)
    {
        right_arm_torque_command.torques(i) = tauDes[15 + i];
        left_arm_torque_command.torques(i) = tauDes[19 + i];
    }

    for (int i = 0; i < 3; i++)
    {
        right_arm_torque_command.torques(4 + i) = tauDes[23 + i];
        left_arm_torque_command.torques(4 + i) = tauDes[26 + i];
    }

    right_arm_torque.write(right_arm_torque_command);
    left_arm_torque.write(left_arm_torque_command);
    right_leg_torque.write(right_leg_torque_command);
    left_leg_torque.write(left_leg_torque_command);
    torso_torque.write(torso_torque_command);
#endif  
}

//  @brief: stops the component (update hook wont be  called anymore)
void Walking::stopHook() 
{
    
}

//  @brief: cleaning the component data
void Walking::cleanupHook() 
{
    
}

double Walking::getSimulationTime() 
{
    #ifndef REAL_ROBOT
        return rtt_rosclock::host_now().toSec();
    #else
        return 1E-9 * os::TimeService::ticks2nsecs(os::TimeService::Instance()->getTicks());        
    #endif
}

#ifdef REAL_ROBOT
bool Walking::attachToRobot(const string &robot_name)
{
    _robot_name = robot_name;
    log(Info)<<"Robot name: "<<_robot_name<<endlog();

    task_ptr = this->getPeer(robot_name);
    if(!task_ptr)
    {
        log(Error)<<"Can not getPeer("<<robot_name<<")"<<endlog();
        return false;
    }

    log(Info)<<"Found Peer "<<robot_name<<endlog();

    OperationCaller<std::map<std::string, std::vector<std::string> >(void) > getKinematicChainsAndJoints = task_ptr->getOperation("getKinematicChainsAndJoints");

    _map_kin_chains_joints = getKinematicChainsAndJoints();

    std::map<std::string, std::vector<std::string> >::iterator it;
    
    for(it = _map_kin_chains_joints.begin(); it != _map_kin_chains_joints.end(); it++)
    {
        std::string kin_chain_name = it->first;
        std::vector<std::string> joint_names = it->second;

        _kinematic_chains_feedback_ports[kin_chain_name] = boost::shared_ptr<RTT::InputPort<rstrt::robot::JointState> >(new RTT::InputPort<rstrt::robot::JointState>(kin_chain_name+"_"+"JointFeedback"));

        this->addPort(*(_kinematic_chains_feedback_ports.at(kin_chain_name))).doc(kin_chain_name+"_"+"JointFeedback port");

        _kinematic_chains_feedback_ports.at(kin_chain_name)->connectTo(task_ptr->ports()->getPort(kin_chain_name+"_"+"JointFeedback"));

        rstrt::robot::JointState tmp(joint_names.size());

        _kinematic_chains_joint_state_map[kin_chain_name] = tmp;
        
        RTT::log(RTT::Info)<<"Added "<<kin_chain_name<<" port and data feedback"<<RTT::endlog();

        _kinematic_chains_output_ports[kin_chain_name] = boost::shared_ptr<RTT::OutputPort<rstrt::kinematics::JointAngles> >(new RTT::OutputPort<rstrt::kinematics::JointAngles>(kin_chain_name+"_"+"JointPositionCtrl"));

        this->addPort(*(_kinematic_chains_output_ports.at(kin_chain_name))).doc(kin_chain_name+"_"+"JointPositionCtrl port");

        _kinematic_chains_output_ports.at(kin_chain_name)->connectTo(task_ptr->ports()->getPort(kin_chain_name+"_"+"JointPositionCtrl"));


        _kinematic_chains_output_voltage_ports[kin_chain_name] = boost::shared_ptr<RTT::OutputPort<rstrt::kinematics::JointAngles> >(new RTT::OutputPort<rstrt::kinematics::JointAngles>(kin_chain_name+"_"+"JointPositionCtrl_VoltageOffset"));

        this->addPort(*(_kinematic_chains_output_voltage_ports.at(kin_chain_name))). doc(kin_chain_name+"_"+"JointPositionCtrl VoltageOffset port");

        _kinematic_chains_output_voltage_ports.at(kin_chain_name)->connectTo(task_ptr->ports()->getPort(kin_chain_name+"_"+"JointPositionCtrl_VoltageOffset"));


        _kinematic_chains_output_torques_ports[kin_chain_name] = boost::shared_ptr<RTT::OutputPort<rstrt::dynamics::JointTorques> >(new RTT::OutputPort<rstrt::dynamics::JointTorques>(kin_chain_name+"_"+"JointTorqueCtrl"));

        this->addPort(*(_kinematic_chains_output_torques_ports.at(kin_chain_name))).doc(kin_chain_name+"_"+"JointTorqueCtrl port");

        _kinematic_chains_output_torques_ports.at(kin_chain_name)->connectTo(task_ptr->ports()->getPort(kin_chain_name+"_"+"JointTorqueCtrl"));

        rstrt::kinematics::JointAngles tmp2(joint_names.size());

        _kinematic_chains_desired_joint_state_map[kin_chain_name] = tmp2;

        _kinematic_chains_desired_joint_voltage_offset_map[kin_chain_name] = tmp2;

        rstrt::dynamics::JointTorques tmp3(joint_names.size());

        _kinematic_chains_desired_joint_torque_map[kin_chain_name] = tmp3;

        RTT::log(RTT::Info)<<"Added "<<kin_chain_name<<" port and data input"<<RTT::endlog();


        _map_chain_trj_time[kin_chain_name] = 0.0;
        _map_chain_start_trj[kin_chain_name] = false;
        _map_chain_q0[kin_chain_name] = rstrt::robot::JointState(joint_names.size());


        _map_chain_start_voltage_trj[kin_chain_name] = false;

        _map_chain_start_torque_trj[kin_chain_name] = false;
    }

    return true;
}

void Walking::getJointLimits(std::map<std::string, std::pair<double, double>>& map)
{
    std::map<std::string, std::vector<std::string> >::iterator it;
    for(it = _map_kin_chains_joints.begin(); it != _map_kin_chains_joints.end(); it++)
    {
        std::string chain_name = it->first;
        std::vector<std::string> joint_names = it->second;
        RTT::log(RTT::Info)<<"Joint Limits for chain "<<chain_name<<RTT::endlog();
        for(unsigned int i = 0; i < joint_names.size(); ++i)
        {
            boost::shared_ptr<const urdf::Joint> joint = _urdf_model->getJoint(joint_names[i]);
            double lower = joint->limits->lower;
            double upper = joint->limits->upper;

            map[joint_names[i]] = std::pair<double,double>(lower, upper);

            RTT::log(RTT::Info)<<"  "<<joint_names[i]<<" ["<<lower<<", "<<upper<<"]"<<RTT::endlog();
        }
    }
}

bool Walking::loadURDFAndSRDF(const std::string &URDF_path, const std::string &SRDF_path)
{
    _urdf_path = URDF_path;
    _srdf_path = SRDF_path;

    RTT::log(RTT::Info)<<"URDF path: "<<_urdf_path<<RTT::endlog();
    RTT::log(RTT::Info)<<"SRDF path: "<<_srdf_path<<RTT::endlog();

    XBot::XBotCoreModel xbot_model;
    bool models_loaded = xbot_model.init(_urdf_path, _srdf_path);
    _urdf_model = xbot_model.get_urdf_model();

    if(models_loaded)
    {
        RTT::log(RTT::Info)<<"Model name: "<<_urdf_model->getName()<<RTT::endlog();

        std::map<std::string, boost::shared_ptr<urdf::Joint> > joint_map = _urdf_model->joints_;
        std::map<std::string, boost::shared_ptr<urdf::Joint> >::iterator it;
        RTT::log(RTT::Info)<<"Joints: "<<RTT::endlog();
        for(it = joint_map.begin(); it != joint_map.end(); it++)
        {
            if(it->second->type != urdf::Joint::FIXED){
                _joint_list.push_back(it->first);
                RTT::log(RTT::Info)<<"  "<<_joint_list.back()<<RTT::endlog();}
        }
        RTT::log(RTT::Info)<<"Total number of joints is "<<_joint_list.size()<<RTT::endlog();
    }


    getJointLimits(_map_joint_limits);

    return models_loaded;
}
#endif

// This macro, as you can see, creates the component. Every component should have this!
// talk about the way it differs from the first component
ORO_CREATE_COMPONENT_LIBRARY()ORO_LIST_COMPONENT_TYPE(Walking)
