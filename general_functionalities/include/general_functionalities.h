#pragma once
#include <Eigen/Dense>
#include <hardware_interface/robot_hw.h>
#include <franka_hw/franka_state_interface.h>
#include <franka_hw/franka_model_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <panda_ecat_comm.h>

namespace general_functionalities
{
    class EEPoleBaseFrameExtWrenchComputation
    {
    public:
        Eigen::Matrix<double,3,3> s_R_0_initial_;
        Eigen::Matrix<double,3,3> s_R_EE_ = Eigen::Matrix3d::Identity();
        
        Eigen::Matrix<double, 6, 1> computeEEPoleBaseFrameExtWrench(const Eigen::Matrix<double, 4, 4>& O_T_EE, const Eigen::Matrix<double, 6, 1>& sensor_data, bool bias_checked, bool bias_error, bool ecat_error);
    };
    
    class locking
    {
    public:
        bool locked_ = true;
        double force_threshold_ = 0.6;
        int unlocking_counter_ = 0;
        int locking_counter_ = 0;
        int unlocking_counter_duration_ = 200;
        int locking_counter_duration_ = 500;

        void locking_unlocking(Eigen::Matrix<double,3,1> F_ext_O);
    };
    
    
    class initial_operations {
        public:
            bool initFrankaVelFT(hardware_interface::RobotHW* robot_hw, std::unique_ptr<franka_hw::FrankaStateHandle>* state_handle, std::unique_ptr<franka_hw::FrankaModelHandle>* model_handle,std::vector<hardware_interface::JointHandle>* joint_handles, panda_ecat_comm::ecatCommATIAxiaFTSensor& FT_sensor, EEPoleBaseFrameExtWrenchComputation& external_force_computation, int filter_level);
            void check_initial_bias(panda_ecat_comm::ecatCommATIAxiaFTSensor& FT_sensor);
            bool bias_checked = false;
            bool bias_error = false;
            int filter_level_;
    };
    
    
    
    
    
}
