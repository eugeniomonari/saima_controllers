#pragma once

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <franka_hw/franka_state_interface.h>
#include <franka_hw/franka_model_interface.h>
#include <panda_ecat_comm.h>
#include <data_extraction.h>
#include <general_functionalities.h>

namespace trial_controller_velocity
{
    class TrialControllerVelocity : public controller_interface::MultiInterfaceController<hardware_interface::VelocityJointInterface, franka_hw::FrankaStateInterface,franka_hw::FrankaModelInterface>
    {
    public: 
        bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
        void update(const ros::Time&, const ros::Duration& period) override;
        void stopping(const ros::Time&) override;
    private:
        std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
        std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
        std::vector<hardware_interface::JointHandle> joint_handles_;
        panda_ecat_comm::ecatCommATIAxiaFTSensor FT_sensor;
        typedef std::tuple<std::array<double,6>,std::array<double,7>,std::array<double,7>,std::array<double,7>,std::array<double,7>,std::array<double,1>,std::array<double,1>> custom_data_t;
        typedef data_extraction::tuple_cat_t<data_extraction::state_model_data_t,custom_data_t> state_model_custom_data_t;
        data_extraction::DataExtraction<state_model_custom_data_t> data_extraction_;
        general_functionalities::locking lockingFunction;
        double T_ = 0.001;
        double mass[6] = {3,3,3,0.3,0.3,0.3};
        double damping[6] = {10,10,10,0.8,0.8,0.8};
        Eigen::Matrix<double,6,1> F_ext_O_lowpass_prev_;
        double kDeltaT = 1e-3;
        double kLimitEps = 1e-3;
        double kTolNumberPacketsLost = 3.0;
        std::array<double, 7> kMaxJointJerk{
            {7500.0 - kLimitEps, 3750.0 - kLimitEps, 5000.0 - kLimitEps, 6250.0 - kLimitEps,
            7500.0 - kLimitEps, 10000.0 - kLimitEps, 10000.0 - kLimitEps}};
        std::array<double, 7> kMaxJointAcceleration{
            {15.0000 - kLimitEps, 7.500 - kLimitEps, 10.0000 - kLimitEps, 12.5000 - kLimitEps,
            15.0000 - kLimitEps, 20.0000 - kLimitEps, 20.0000 - kLimitEps}};
        std::array<double, 7> kMaxJointVelocity{
            {2.1750 - kLimitEps - kTolNumberPacketsLost * kDeltaT * kMaxJointAcceleration[0],
            2.1750 - kLimitEps - kTolNumberPacketsLost* kDeltaT* kMaxJointAcceleration[1],
            2.1750 - kLimitEps - kTolNumberPacketsLost* kDeltaT* kMaxJointAcceleration[2],
            2.1750 - kLimitEps - kTolNumberPacketsLost* kDeltaT* kMaxJointAcceleration[3],
            2.6100 - kLimitEps - kTolNumberPacketsLost* kDeltaT* kMaxJointAcceleration[4],
            2.6100 - kLimitEps - kTolNumberPacketsLost* kDeltaT* kMaxJointAcceleration[5],
            2.6100 - kLimitEps - kTolNumberPacketsLost* kDeltaT* kMaxJointAcceleration[6]}};
        Eigen::Matrix<double,2,1> setSafeVelocities(double max_velocity,double max_acceleration,double max_jerk,double last_commanded_velocity,double last_commanded_acceleration);
        general_functionalities::EEPoleBaseFrameExtWrenchComputation external_force_computation;
        general_functionalities::initial_operations initOperations;
    };
        
}