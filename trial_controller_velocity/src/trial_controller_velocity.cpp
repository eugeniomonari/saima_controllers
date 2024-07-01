#include "trial_controller_velocity.h"
#include <Eigen/Dense>
#include <pluginlib/class_list_macros.h>
#include <franka/rate_limiting.h>
#include <optimizer1_bindings.hpp>

namespace trial_controller_velocity
{
    bool TrialControllerVelocity::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle)
    {
        initOperations.initFrankaVelFT(robot_hw,&state_handle_,&model_handle_,&joint_handles_,FT_sensor,external_force_computation,7);
        F_ext_O_lowpass_prev_.setZero();
        return true;
    }
    
    void TrialControllerVelocity::update(const ros::Time&, const ros::Duration& period)
    {
        initOperations.check_initial_bias(FT_sensor);
        data_extraction_.started = true;
        Eigen::Matrix<double,6,1> F_ext_s;
        F_ext_s = FT_sensor.get_FT_sensor_data();
        franka::RobotState robot_state = state_handle_->getRobotState();
        Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
        Eigen::Map<Eigen::Matrix<double, 7, 1>> dq_d(robot_state.dq_d.data());
        Eigen::Map<Eigen::Matrix<double, 7, 1>> ddq_d(robot_state.ddq_d.data());
        Eigen::Map<Eigen::Matrix<double, 4, 4>> O_T_EE(robot_state.O_T_EE.data());
        std::array<double, 42> J_vector = model_handle_->getZeroJacobian(franka::Frame::kEndEffector,state_handle_->getRobotState().q_d,state_handle_->getRobotState().F_T_EE,state_handle_->getRobotState().EE_T_K);
        Eigen::Map<Eigen::Matrix<double, 6, 7>> J(J_vector.data());
        std::array<double, 49> B_vector = model_handle_->getMass();
        Eigen::Map<Eigen::Matrix<double, 7, 7>> B(B_vector.data());
        
        Eigen::Matrix<double,3,3> EE_R_O;
        EE_R_O = O_T_EE.block(0,0,3,3).transpose();
        Eigen::Matrix<double,6,1> F_ext_O = external_force_computation.computeEEPoleBaseFrameExtWrench(O_T_EE,F_ext_s,initOperations.bias_checked,initOperations.bias_error,FT_sensor.ecat_error);
        lockingFunction.locking_unlocking(F_ext_O.head(3));
        bool only_transl = true;
        if (only_transl) {
            F_ext_O[3] = 0;
            F_ext_O[4] = 0;
            F_ext_O[5] = 0;
        }
        
        Eigen::Matrix<double,7,1> dq_c;
        dq_c.setZero();
        if (!lockingFunction.locked_) {
            bool only_damping = true;
            if (!only_damping) {
                Eigen::Matrix<double,6,1> v_d = J*dq_d;
                Eigen::Matrix<double,6,1> v_c;
                v_c.setZero();
                for (size_t i = 0; i < 6; i++) {
                    v_c[i] = mass[i]/(mass[i]+T_*damping[i])*v_d[i] + T_/(mass[i] + T_*damping[i])*F_ext_O[i];
                }
                Eigen::Matrix<double,7,6> J_pseudoinv = B.inverse()*J.transpose()*(J*B.inverse()*J.transpose()).inverse();
                dq_c = J_pseudoinv*v_c;
            }
            else {
                Eigen::Matrix<double,6,1> F_ext_0_lowpass;
                F_ext_0_lowpass.setZero();
                for (size_t i=0;i<6;++i){
                    F_ext_0_lowpass[i] = franka::lowpassFilter(0.001,F_ext_O[i],F_ext_O_lowpass_prev_[i],1);
                }
                F_ext_O_lowpass_prev_ = F_ext_0_lowpass;
                Eigen::Matrix<double,6,1> v_c;
                v_c.setZero();
                for (size_t i = 0; i < 6; i++) {
                    v_c[i] = F_ext_0_lowpass[i]/damping[i];
                }
                Eigen::Matrix<double,7,6> J_pseudoinv = B.inverse()*J.transpose()*(J*B.inverse()*J.transpose()).inverse();
                dq_c = J_pseudoinv*v_c;
            }
        }
        Eigen::Matrix<double,7,1> dq_max_safe;
        Eigen::Matrix<double,7,1> dq_min_safe;
        Eigen::Matrix<double,2,1> velocity_limits;
        bool limits_violated = false;
        for (size_t i = 0; i < 7; i++) {
            velocity_limits = setSafeVelocities(kMaxJointVelocity[i],kMaxJointAcceleration[i],kMaxJointJerk[i],dq_d[i],ddq_d[i]);
            dq_min_safe[i] = velocity_limits[0];
            dq_max_safe[i] = velocity_limits[1];
            if (!(dq_c[i] <= dq_max_safe[i] && dq_c[i] >= dq_min_safe[i])) {
                limits_violated = true;
            }
        }
        Eigen::Matrix<double,7,1> dq_c_lim;
        dq_c_lim.setZero();
        if (limits_violated) {
            for (size_t i = 0; i < 7; i++) {
                dq_c_lim(i) = std::max(std::min(dq_c[i], dq_max_safe[i]), dq_min_safe[i]);
            }
        }
        else {
            dq_c_lim = dq_c;
        }
        for (size_t i = 0; i < 7; i++) {
            joint_handles_[i].setCommand(dq_c_lim(i));
        }
                
        std::vector<Eigen::VectorXd> custom_data(7);
        custom_data[0] = F_ext_s;
        custom_data[1] = dq_c;
        custom_data[2] = dq_c_lim;
        custom_data[3] = dq_min_safe;
        custom_data[4] = dq_max_safe;
        Eigen::Matrix<double,1,1> limits_violated_vector;
        limits_violated_vector[0] = lockingFunction.locked_;
        custom_data[5] = limits_violated_vector;
        Eigen::Matrix<double,1,1> locked_vector;
        locked_vector[0] = lockingFunction.locked_;
        custom_data[6] = locked_vector;
        data_extraction_.update_data(&state_handle_,&model_handle_,custom_data);
    }
    
    void TrialControllerVelocity::stopping(const ros::Time&)
    {
        std::vector<std::string> custom_header_values{"F_ext_s","dq_c","dq_c_lim","dq_min_safe","dq_max_safe","limits_violated","locked"};
        data_extraction_.write_data_to_csv(custom_header_values);
        data_extraction_.started = false;
    }
    
    Eigen::Matrix<double,2,1> TrialControllerVelocity::setSafeVelocities(double max_velocity,double max_acceleration,double max_jerk,double last_commanded_velocity,double last_commanded_acceleration) {
        double safe_max_acceleration = std::max(std::min({
            (max_jerk / max_acceleration) * (max_velocity - last_commanded_velocity), max_acceleration,last_commanded_acceleration+max_jerk*kDeltaT}),last_commanded_acceleration-max_jerk*kDeltaT);
        double safe_min_acceleration = std::min(std::max({
            (max_jerk / max_acceleration) * (-max_velocity - last_commanded_velocity), -max_acceleration,last_commanded_acceleration-max_jerk*kDeltaT}),last_commanded_acceleration+max_jerk*kDeltaT);
        Eigen::Matrix<double,2,1> safe_velocities;
        safe_velocities.setZero();
        safe_velocities[0] = last_commanded_velocity+safe_min_acceleration*kDeltaT;
        safe_velocities[1] = last_commanded_velocity+safe_max_acceleration*kDeltaT;
        return safe_velocities;
    }
}

PLUGINLIB_EXPORT_CLASS(trial_controller_velocity::TrialControllerVelocity,controller_interface::ControllerBase)
