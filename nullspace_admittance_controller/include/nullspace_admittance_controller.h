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
#include <nullspace_admittance_controller/nullspace_admittance_controller_paramConfig.h>
#include <dynamic_reconfigure/server.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <ros/ros.h>

namespace nullspace_admittance_controller
{
    class NullspaceAdmittanceController : public controller_interface::MultiInterfaceController<hardware_interface::VelocityJointInterface, franka_hw::FrankaStateInterface,franka_hw::FrankaModelInterface>
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
//         custom_header_values{"F_ext_EE_0","F_ext_EE_0_lowpass","dq_max_safe","dq_min_safe","J","J_next","B_next","J_pinv_next","v_c","m_tr","d_tr","m_rot","d_rot","limits_violated","locked"};
        typedef std::tuple<std::array<double,6>,std::array<double,6>,std::array<double,7>,std::array<double,7>,std::array<double,42>,std::array<double,42>,std::array<double,49>,std::array<double,42>,std::array<double,6>,std::array<double,1>,std::array<double,1>,std::array<double,1>,std::array<double,1>,std::array<double,1>,std::array<double,1>> custom_data_t;
        typedef data_extraction::tuple_cat_t<data_extraction::state_model_data_t,custom_data_t> state_model_custom_data_t;
        data_extraction::DataExtraction<state_model_custom_data_t> data_extraction_;
        general_functionalities::locking lockingFunction;
        double T_ = 0.001;
        double m_tr_ = 0;
        double m_rot_ = 0;
        double d_tr_ = 0;
        double d_rot_ = 0;
        Eigen::Matrix<double,6,1> F_ext_EE_0_lowpass_prev_;
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
        std::array<double, 7> kMaxJointPosition{
            {2.8973 - kLimitEps - kTolNumberPacketsLost * kDeltaT * kMaxJointVelocity[0],
            1.7628 - kLimitEps - kTolNumberPacketsLost* kDeltaT* kMaxJointVelocity[1],
            2.8973 - kLimitEps - kTolNumberPacketsLost* kDeltaT* kMaxJointVelocity[2],
            -0.0698 - kLimitEps - kTolNumberPacketsLost* kDeltaT* kMaxJointVelocity[3],
            2.8973 - kLimitEps - kTolNumberPacketsLost* kDeltaT* kMaxJointVelocity[4],
            3.7525 - kLimitEps - kTolNumberPacketsLost* kDeltaT* kMaxJointVelocity[5],
            2.8973 - kLimitEps - kTolNumberPacketsLost* kDeltaT* kMaxJointVelocity[6]}};
        std::array<double, 7> kMinJointPosition{
            {-2.8973 - kLimitEps - kTolNumberPacketsLost * kDeltaT * -kMaxJointVelocity[0],
            -1.7628 - kLimitEps - kTolNumberPacketsLost* kDeltaT* -kMaxJointVelocity[1],
            -2.8973 - kLimitEps - kTolNumberPacketsLost* kDeltaT* -kMaxJointVelocity[2],
            -3.0718 - kLimitEps - kTolNumberPacketsLost* kDeltaT* -kMaxJointVelocity[3],
            -2.8973 - kLimitEps - kTolNumberPacketsLost* kDeltaT* -kMaxJointVelocity[4],
            -0.0175 - kLimitEps - kTolNumberPacketsLost* kDeltaT* -kMaxJointVelocity[5],
            -2.8973 - kLimitEps - kTolNumberPacketsLost* kDeltaT* -kMaxJointVelocity[6]}};
        Eigen::Matrix<double,2,1> setSafeVelocities(double max_position, double min_position, double max_velocity,double max_acceleration,double max_jerk,double last_commanded_position,double last_commanded_velocity,double last_commanded_acceleration);
        general_functionalities::EEPoleBaseFrameExtWrenchComputation external_force_computation;
        general_functionalities::initial_operations initOperations;
        bool only_transl_ = false;
        bool only_x_ = false;
        bool only_damping_ = false;
        double lowpass_filter_cutoff_freq_ = 1000;
        
        std::unique_ptr<dynamic_reconfigure::Server<nullspace_admittance_controller::nullspace_admittance_controller_paramConfig>> dynamic_server_nullspace_admittance_controller_param_;
        ros::NodeHandle dynamic_reconfigure_nullspace_admittance_controller_param_node_;
        void trialControllerVelocityParamCallback(nullspace_admittance_controller::nullspace_admittance_controller_paramConfig& config, uint32_t level);
        bool dyn_params_set = false;
        std::unique_ptr<planning_scene::PlanningScene> planning_scene_;
        general_functionalities::collision_free_command collisionFreeCommand;
    };
        
}
