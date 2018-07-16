#ifndef WALKING_HPP
#define WALKING_HPP


#define COMAN_RIGHT_ARM_DOF_SIZE 7
#define COMAN_LEFT_ARM_DOF_SIZE 7
#define COMAN_RIGHT_LEG_DOF_SIZE 6
#define COMAN_LEFT_LEG_DOF_SIZE 6
#define COMAN_TORSO_DOF_SIZE 3
#define NUM 31

// RTT header files. Might missing some or some be unused
#include <rtt/Port.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Component.hpp>
#include <string>
#include <cmath>

#include <rst-rt/dynamics/JointTorques.hpp>
#include <rst-rt/robot/JointState.hpp>
#include <rst-rt/dynamics/Wrench.hpp>
#include <rst-rt/kinematics/JointAngles.hpp>

//#include <sensor_msgs/Imu.h> // is it real time safe??
#include <rst-rt/kinematics/JointVelocities.hpp>
#include <rst-rt/kinematics/JointAccelerations.hpp>
#include <rst-rt/dynamics/Torques.hpp>
#include <rst-rt/dynamics/JointImpedance.hpp>
#include <rst-rt/dynamics/JointTorques.hpp>
#include <rst-rt/dynamics/Forces.hpp>
#include <rst-rt/robot/JointState.hpp>
#include <cstdlib>
#include <rst-rt/dynamics/Wrench.hpp>
#include <rtt/plugin/PluginLoader.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>
#include "rosgraph_msgs/Clock.h"

#include <rtt/RTT.hpp>
#include <string>
#include <rst-rt/robot/JointState.hpp>
#include <rst-rt/kinematics/JointAngles.hpp>
#include <rst-rt/dynamics/JointTorques.hpp>
#include <rst-rt/dynamics/JointImpedance.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Component.hpp>
#include <rtt/Port.hpp>

#ifdef REAL_ROBOT
#include <XBotCoreModel.h>
#include <urdf/model.h>
#endif


//#include <rtt_roscomm/rtt_rostopic.h>

using namespace RTT;

class Walking: public RTT::TaskContext 
{
    public:
        Walking(std::string const & name);

        // TaskContext methods that are needed for any standard component and
        // should be implemented by user
        bool configureHook();
        bool startHook();
        void updateHook();
        void stopHook();
        void cleanupHook();

        RTT::TaskContext* task_ptr;

        bool attachToRobot(const std::string &robot_name);

        #ifdef REAL_ROBOT
        void getJointLimits(std::map<std::string, std::pair<double, double>>& map);

        bool loadURDFAndSRDF(const std::string &URDF_path, const std::string &SRDF_path);
        #endif

        unsigned int whichComan_ = 1;

    private:
        // Declare ports and their datatypes
        // this time an input port is needed as well

        // Data flow:
        // input ports need dataflow

    //    bool use_real_robot;

        #ifdef REAL_ROBOT
        std::string _urdf_path;
        std::string _srdf_path;
        boost::shared_ptr<urdf::ModelInterface const> _urdf_model;
        std::vector<std::string> _joint_list;
        std::string _robot_name;
        std::map<std::string, std::vector<std::string> > _map_kin_chains_joints;
        #endif

        std::map<std::string, double> _map_chain_trj_time;
        std::map<std::string, rstrt::robot::JointState> _map_chain_q0;
        std::map<std::string, bool> _map_chain_start_trj;
        std::map<std::string, bool> _map_chain_start_voltage_trj;
        std::map<std::string, bool> _map_chain_start_torque_trj;

        RTT::InputPort<rstrt::robot::JointState> left_arm_joint_data;
        RTT::InputPort<rstrt::robot::JointState> right_arm_joint_data;
        RTT::InputPort<rstrt::robot::JointState> right_leg_joint_data;
        RTT::InputPort<rstrt::robot::JointState> left_leg_joint_data;
        RTT::InputPort<rstrt::robot::JointState> torso_joint_data;

        rstrt::robot::JointState left_arm_joint_data_store;
        rstrt::robot::JointState right_arm_joint_data_store;
        rstrt::robot::JointState right_leg_joint_data_store;
        rstrt::robot::JointState left_leg_joint_data_store;
        rstrt::robot::JointState torso_joint_data_store;

        std::map<std::string, boost::shared_ptr<RTT::InputPort<rstrt::robot::JointState> > > _kinematic_chains_feedback_ports;
        std::map<std::string, rstrt::robot::JointState> _kinematic_chains_joint_state_map;

        std::map<std::string, boost::shared_ptr<RTT::OutputPort<rstrt::kinematics::JointAngles> > > _kinematic_chains_output_ports;
        std::map<std::string, boost::shared_ptr<RTT::OutputPort<rstrt::kinematics::JointAngles> > > _kinematic_chains_output_voltage_ports;
        std::map<std::string, boost::shared_ptr<RTT::OutputPort<rstrt::dynamics::JointTorques> > > _kinematic_chains_output_torques_ports;

        std::map<std::string, rstrt::kinematics::JointAngles> _kinematic_chains_desired_joint_state_map;
        std::map<std::string, rstrt::kinematics::JointAngles> _kinematic_chains_desired_joint_voltage_offset_map;
        std::map<std::string, rstrt::dynamics::JointTorques> _kinematic_chains_desired_joint_torque_map;

        RTT::InputPort<rstrt::dynamics::Wrench> right_foot_wrench;
        RTT::InputPort<rstrt::dynamics::Wrench> left_foot_wrench;
        RTT::InputPort<rstrt::dynamics::Wrench> right_arm_wrench;
        RTT::InputPort<rstrt::dynamics::Wrench> left_arm_wrench;

        rstrt::dynamics::Wrench right_foot_wrench_store;
        rstrt::dynamics::Wrench left_foot_wrench_store;
        rstrt::dynamics::Wrench right_arm_wrench_store;
        rstrt::dynamics::Wrench left_arm_wrench_store;

        std::map<std::string, std::pair<double, double>> _map_joint_limits;

        #ifdef REAL_ROBOT
        RTT::InputPort<Eigen::Vector3f> imu_port;
        Eigen::Vector3f imu_data_;
        std::string imu_topic;
        #endif
        

    //    double qSens[NUM], Q0[NUM], dqSens[NUM], qSensAbs[NUM], forceRightAnkle[3], forceLeftAnkle[3], forceRightArm[3], forceLeftArm[3];
    //    double vals[NUM], tauDes[NUM];
    //    double tme;

        //ROS
        RTT::InputPort<sensor_msgs::Imu> _imu_port;
        sensor_msgs::Imu _imu_msg;

        // RTT::InputPort<rosgraph_msgs::TopicStatistics> _topic_port;
        // rosgraph_msgs::TopicStatistics _topic_msg;

        RTT::OutputPort<rstrt::dynamics::JointTorques> left_arm_torque;
        RTT::OutputPort<rstrt::dynamics::JointTorques> right_arm_torque;
        RTT::OutputPort<rstrt::dynamics::JointTorques> right_leg_torque;
        RTT::OutputPort<rstrt::dynamics::JointTorques> left_leg_torque;
        RTT::OutputPort<rstrt::dynamics::JointTorques> torso_torque;

        rstrt::dynamics::JointTorques left_arm_torque_command;
        rstrt::dynamics::JointTorques right_arm_torque_command;
        rstrt::dynamics::JointTorques left_leg_torque_command;
        rstrt::dynamics::JointTorques right_leg_torque_command;
        rstrt::dynamics::JointTorques torso_torque_command;

        RTT::OutputPort<rstrt::kinematics::JointAngles> left_arm_voltage;
        RTT::OutputPort<rstrt::kinematics::JointAngles> right_arm_voltage;
        RTT::OutputPort<rstrt::kinematics::JointAngles> right_leg_voltage;
        RTT::OutputPort<rstrt::kinematics::JointAngles> left_leg_voltage;
        RTT::OutputPort<rstrt::kinematics::JointAngles> torso_voltage;

        rstrt::kinematics::JointAngles left_arm_voltage_command;
        rstrt::kinematics::JointAngles right_arm_voltage_command;
        rstrt::kinematics::JointAngles left_leg_voltage_command;
        rstrt::kinematics::JointAngles right_leg_voltage_command;
        rstrt::kinematics::JointAngles torso_voltage_command;

        void preparePorts();

        // Actuall joint command to be sent over port:
        
        // helpers:
        double getSimulationTime();

        void toTrans(Eigen::Vector3f E_imu, double Trans[3][3])
        {
            double thp, thr, thy;

            thp = E_imu[0];
            thr = E_imu[1];
            thy = E_imu[2];

            double c1 = cos(-thr); // 1
            double c2 = cos(-thp); // 2
            double c3 = cos(-thy); // 3
            double s1 = sin(-thr);
            double s2 = sin(-thp);            
            double s3 = sin(-thy);

            Trans[0][0] = c2*c3;
            Trans[0][1] = c1*s3 + c3*s1*s2;
            Trans[0][2] = s1*s3 - c1*c3*s2;
            Trans[1][0] = -c2*s3;
            Trans[1][1] = c1*c3 - s1*s2*s3;
            Trans[1][2] = c3*s1 + c1*s2*s3;
            Trans[2][0] = s2;
            Trans[2][1] = -c2*s1;
            Trans[2][2] = c1*c2;
        }

        // operations:
        // are class methods that can be called from deployer
        

        // rstrt::kinematics::JointAngles sin_traj(const rstrt::robot::JointState& q0, double amplitude, double t, double period)
        // {
        //     rstrt::kinematics::JointAngles tmp(q0.angles.rows());
        //     for(unsigned int i = 0; i < q0.angles.rows(); ++i)
        //         tmp.angles[i] = sin_traj(q0.angles[i], amplitude, t, period);
        //     return tmp;
        // }

        

};

#endif // SIMPLERTTCOMPONENT_HPP
