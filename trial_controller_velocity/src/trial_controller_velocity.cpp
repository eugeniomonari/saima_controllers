#include "trial_controller_velocity.h"
#include <Eigen/Dense>
#include <pluginlib/class_list_macros.h>
#include <franka/rate_limiting.h>

#include <qpOASES.hpp>

namespace trial_controller_velocity
{
    bool TrialControllerVelocity::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle)
    {
        if (!node_handle.getParam("only_damping", only_damping_)) {
            ROS_ERROR_STREAM("HandGuidanceController: could not get only_damping from parameter server");
            return false;
        }
        if (!only_damping_) {
            if (!node_handle.getParam("small_mass", small_mass_)) {
                ROS_ERROR_STREAM("HandGuidanceController: could not get small_mass from parameter server");
                return false;
            }
        }
        if (only_damping_ || (!only_damping_ && small_mass_)) {
            initOperations.initFrankaVelFT(robot_hw,&state_handle_,&model_handle_,&joint_handles_,FT_sensor,external_force_computation,7);
            if (!node_handle.getParam("only_x", only_x_)) {
                ROS_ERROR_STREAM("HandGuidanceController: could not get only_x from parameter server");
                return false;
            }
            if (!node_handle.getParam("m_tr_no_mass", m_tr_)) {
                ROS_ERROR_STREAM("HandGuidanceController: could not get m_tr_no_mass from parameter server");
                return false;
            }
            if (!node_handle.getParam("m_rot_no_mass", m_rot_)) {
                ROS_ERROR_STREAM("HandGuidanceController: could not get m_rot_no_mass from parameter server");
                return false;
            }
            if (!node_handle.getParam("d_tr_no_mass", d_tr_)) {
                ROS_ERROR_STREAM("HandGuidanceController: could not get d_tr_no_mass from parameter server");
                return false;
            }
            if (!node_handle.getParam("d_rot_no_mass", d_rot_)) {
                ROS_ERROR_STREAM("HandGuidanceController: could not get d_rot_no_mass from parameter server");
                return false;
            }
        }
        else {
            initOperations.initFrankaVelFT(robot_hw,&state_handle_,&model_handle_,&joint_handles_,FT_sensor,external_force_computation,0);
            if (!node_handle.getParam("m_tr", m_tr_)) {
                ROS_ERROR_STREAM("HandGuidanceController: could not get m_tr from parameter server");
                return false;
            }
            if (!node_handle.getParam("m_rot", m_rot_)) {
                ROS_ERROR_STREAM("HandGuidanceController: could not get m_rot from parameter server");
                return false;
            }
            if (!node_handle.getParam("d_tr", d_tr_)) {
                ROS_ERROR_STREAM("HandGuidanceController: could not get d_tr from parameter server");
                return false;
            }
            if (!node_handle.getParam("d_rot", d_rot_)) {
                ROS_ERROR_STREAM("HandGuidanceController: could not get d_rot from parameter server");
                return false;
            }
        }
        F_ext_EE_0_lowpass_prev_.setZero();

        if (!node_handle.getParam("only_transl", only_transl_)) {
            ROS_ERROR_STREAM("HandGuidanceController: could not get only_transl from parameter server");
            return false;
        }
        
        dynamic_reconfigure_trial_controller_velocity_param_node_ =
        ros::NodeHandle("dynamic_reconfigure_trial_controller_velocity_param_node");
        dynamic_server_trial_controller_velocity_param_ = std::make_unique<dynamic_reconfigure::Server<trial_controller_velocity::trial_controller_velocity_paramConfig>>( dynamic_reconfigure_trial_controller_velocity_param_node_);
        dynamic_server_trial_controller_velocity_param_->setCallback(boost::bind(&TrialControllerVelocity::trialControllerVelocityParamCallback, this, _1, _2));
        
        return true;
    }
    
    void TrialControllerVelocity::update(const ros::Time&, const ros::Duration& period)
    {
        initOperations.check_initial_bias(FT_sensor);
        data_extraction_.started = true;
        Eigen::Matrix<double,6,1> F_ext_S_s;
        F_ext_S_s = FT_sensor.get_FT_sensor_data();
        franka::RobotState robot_state = state_handle_->getRobotState();
        Eigen::Map<Eigen::Matrix<double, 7, 1>> dq_d(robot_state.dq_d.data());
        Eigen::Map<Eigen::Matrix<double, 7, 1>> ddq_d(robot_state.ddq_d.data());
        Eigen::Map<Eigen::Matrix<double, 4, 4>> O_T_EE(robot_state.O_T_EE.data());
        Eigen::Matrix<double,6,1> F_ext_EE_0 = external_force_computation.computeEEPoleBaseFrameExtWrench(O_T_EE,F_ext_S_s,initOperations.bias_checked,initOperations.bias_error,FT_sensor.ecat_error);
        lockingFunction.locking_unlocking(F_ext_EE_0.head(3));
        if (only_transl_) {
            if (only_x_) {
                F_ext_EE_0[1] = 0;
                F_ext_EE_0[2] = 0;
            }
            F_ext_EE_0[3] = 0;
            F_ext_EE_0[4] = 0;
            F_ext_EE_0[5] = 0;
        }
        
        Eigen::Matrix<double,7,1> dq_max_safe;
        Eigen::Matrix<double,7,1> dq_min_safe;
        std::array<double, 42> J_vector = model_handle_->getZeroJacobian(franka::Frame::kEndEffector,state_handle_->getRobotState().q_d,state_handle_->getRobotState().F_T_EE,state_handle_->getRobotState().EE_T_K);
        Eigen::Map<Eigen::Matrix<double, 6, 7>> J(J_vector.data());
        Eigen::Matrix<double,7,1> B = Eigen::Matrix<double,7,1>::Zero();
        Eigen::FullPivLU<Eigen::MatrixXd> lu(J);
        B = lu.kernel();
        Eigen::Matrix<double,2,1> velocity_limits;
        for (size_t i = 0; i < 7; i++) {
            velocity_limits = setSafeVelocities(kMaxJointVelocity[i],kMaxJointAcceleration[i],kMaxJointJerk[i],dq_d[i],ddq_d[i]);
            dq_min_safe[i] = velocity_limits[0];
            dq_max_safe[i] = velocity_limits[1];
        }
        Eigen::Matrix<double,6,1> F_ext_EE_0_lowpass;
        F_ext_EE_0_lowpass.setZero();
        Eigen::Matrix<double,7,1> dq_c;
        dq_c.setZero();
        Eigen::Matrix<double,7,1> dq_c_opt = Eigen::Matrix<double,7,1>::Zero();
        qpOASES::real_t solution[8] = {1/m_tr_,0,0,0,0,0,0,0};
//         double u[6] = {m_tr_, d_tr_, 0.0, 0.0, 0.0, 0.0};
        Eigen::Matrix<double,7,1> dq_c_lim;
        bool limits_violated = false;
        for (size_t i=0;i<6;++i){
            F_ext_EE_0_lowpass[i] = franka::lowpassFilter(0.001,F_ext_EE_0[i],F_ext_EE_0_lowpass_prev_[i],1);
        }
        if (lockingFunction.locked_) {
            dq_c.setZero();
            for (size_t i = 0; i < 7; i++) {
                if (!(dq_c[i] <= dq_max_safe[i] && dq_c[i] >= dq_min_safe[i])) {
                    limits_violated = true;
                    dq_c_lim(i) = std::max(std::min(dq_c[i], dq_max_safe[i]), dq_min_safe[i]);
                }
                else {
                    dq_c_lim(i) = dq_c(i);
                }
            }
        }
        else {
            if (only_damping_) {
                Eigen::Matrix<double,6,1> v_c;
                for (size_t i = 0; i < 6; i++) {
                    if (i < 3) {
                        v_c[i] = F_ext_EE_0_lowpass[i]/d_tr_;
                    }
                    else {
                        v_c[i] = F_ext_EE_0_lowpass[i]/d_rot_;
                    }
                }
                Eigen::Matrix<double,7,6> J_pinv;
                
                std::array<double, 49> B_vector = model_handle_->getMass();
                Eigen::Map<Eigen::Matrix<double, 7, 7>> B(B_vector.data());
                J_pinv = B.inverse()*J.transpose()*(J*B.inverse()*J.transpose()).inverse();
                dq_c = J_pinv*v_c;
                for (size_t i = 0; i < 7; i++) {
                    if (!(dq_c[i] <= dq_max_safe[i] && dq_c[i] >= dq_min_safe[i])) {
                        limits_violated = true;
                    }
                }
                if (limits_violated) {
                    std::cout << "limits_violated" << std::endl;
                    for (size_t i = 0; i < 7; i++) {
                        dq_c_lim(i) = std::max(std::min(dq_c[i], dq_max_safe[i]), dq_min_safe[i]);
                    }
                }
                else {
                    dq_c_lim = dq_c;
                }
            }
            else {
                Eigen::Matrix<double,6,1> v_c;
                v_c.setZero();
                std::array<double, 42> J_vector = model_handle_->getZeroJacobian(franka::Frame::kEndEffector,state_handle_->getRobotState().q_d,state_handle_->getRobotState().F_T_EE,state_handle_->getRobotState().EE_T_K);
                Eigen::Map<Eigen::Matrix<double, 6, 7>> J(J_vector.data());
                Eigen::Matrix<double,6,1> v_prev = J*dq_d;
                if (small_mass_) {
                    for (size_t i = 0; i < 6; i++) {
                        if (i < 3) {
                            v_c[i] = v_prev[i]+1/m_tr_*(-d_tr_*v_prev[i]*T_+F_ext_EE_0_lowpass(i)*T_);
                        }
                        else {
                            v_c[i] = F_ext_EE_0_lowpass[i]/d_rot_;
                        }
                    }
                }
                else {
                    for (size_t i = 0; i < 6; i++) {
                        if (i < 3) {
                            v_c[i] = v_prev[i]+1/m_tr_*(-d_tr_*v_prev[i]*T_+F_ext_EE_0(i)*T_);
                        }
                        else {
                            v_c[i] = v_prev[i]+1/m_rot_*(-d_rot_*v_prev[i]*T_+F_ext_EE_0(i)*T_);
                        }
                    }
                }
                std::array<double, 49> B_vector = model_handle_->getMass();
                Eigen::Map<Eigen::Matrix<double, 7, 7>> B_mat(B_vector.data());
                Eigen::Matrix<double,7,6> J_pinv = B_mat.inverse()*J.transpose()*(J*B_mat.inverse()*J.transpose()).inverse();
                dq_c = J_pinv*v_c;
                for (size_t i = 0; i < 7; i++) {
                    if (!(dq_c[i] <= dq_max_safe[i] && dq_c[i] >= dq_min_safe[i])) {
                        limits_violated = true;
                    }
                }
                if (limits_violated) {
//                     double k1 = 1;
//                     double k2 = 1;
//                     double k3 = 1000;
//                     qpOASES::real_t H[8*8];
//                     for (size_t i = 0; i < 8; i++) {
//                         for (size_t j = 0; j < 8; j++) {
//                             if (i==j) {
//                                 if (i == 1) {
//                                     H[i*8+j] = k2;
//                                 }
//                                 else if (i > 1) {
//                                     H[i*8+j] = k3;
//                                 }
//                             }
//                             else {
//                                 H[i*8+j] = 0;
//                             }
//                         }
//                     }
//                     qpOASES::real_t g[8] = {k1,0,0,0,0,0,0,0};
//                     Eigen::Matrix<double,7,8> A_eigen = Eigen::Matrix<double,7,8>::Zero();
//                     A_eigen.block(0,0,7,1) = J_pinv.block(0,0,7,3)*(d_tr_*v_prev.head(3)*T_-F_ext_EE_0_lowpass.head(3)*T_);
//                     A_eigen.block(0,1,7,1) = b;
//                     A_eigen.block(0,2,7,6) = J_pinv;
//                     qpOASES::real_t A[7*8];
//                     for (size_t i = 0; i < 7; i++) {
//                         for (size_t j = 0; j < 8; j++) {
//                             A[i*8+j] = A_eigen(i,j);
//                         }
//                     }
//                     Eigen::Matrix<double,7,1> ubA_eigen = Eigen::Matrix<double,7,1>::Zero();
//                     ubA_eigen = dq_max_safe - J_pinv.block(0,0,7,3)*v_prev.head(3)-J_pinv.block(3,0,7,3)*F_ext_EE_0_lowpass.tail(3)/d_rot_;
//                     qpOASES::real_t ubA[7];
//                     Eigen::Matrix<double,7,1> lbA_eigen = Eigen::Matrix<double,7,1>::Zero();
//                     lbA_eigen = dq_min_safe - J_pinv.block(0,0,7,3)*v_prev.head(3)-J_pinv.block(3,0,7,3)*F_ext_EE_0_lowpass.tail(3)/d_rot_;
//                     qpOASES::real_t lbA[7];
//                     for (size_t i = 0; i < 7; i++) {
//                         ubA[i] = ubA_eigen[i];
//                         lbA[i] = lbA_eigen[i];
//                     }
//                     int nWSR = 100;
//                     qpOASES::real_t lb[8] = {-1/m_tr_,-qpOASES::INFTY,-qpOASES::INFTY,-qpOASES::INFTY,-qpOASES::INFTY,-qpOASES::INFTY,-qpOASES::INFTY,-qpOASES::INFTY};
//                     qpOASES::real_t ub[8] = {-1/100, qpOASES::INFTY,qpOASES::INFTY,qpOASES::INFTY,qpOASES::INFTY,qpOASES::INFTY,qpOASES::INFTY,qpOASES::INFTY};
//                     qpOASES::SQProblem opt_problem( 8,7 );
//                     qpOASES::returnValue return_value = opt_problem.init( H,g,A,lb,ub,lbA,ubA, nWSR,0,solution );
// // //                     std::cout << qpOASES::getSimpleStatus(return_value) << std::endl;
// //                     
// //                     
//                     opt_problem.getPrimalSolution(solution);
//                     Eigen::Matrix<double,6,1> delta = Eigen::Matrix<double,6,1>::Zero();
//                     for (size_t i = 0; i < 6; i++) {
//                         delta(i) = solution[2+i];
//                     }
//                     Eigen::Matrix<double,6,1> v_adm_lim = Eigen::Matrix<double,6,1>::Zero();
//                     std::cout << -1/solution[0] << std::endl;
//                     v_adm_lim.head(3) = v_prev.head(3)-solution[0]*(-d_tr_*v_prev.head(3)*T_+F_ext_EE_0_lowpass.head(3)*T_);
//                     v_adm_lim.tail(3) = F_ext_EE_0_lowpass.tail(3)/d_rot_;
//                     dq_c_opt = J_pinv*(v_adm_lim+delta)+b*solution[1];
                    for (size_t i = 0; i < 7; i++) {
                        dq_c_lim(i) = std::max(std::min(dq_c[i], dq_max_safe[i]), dq_min_safe[i]);
                    }
//                     dq_c_lim = dq_c_opt;
                }
                else {
                    dq_c_lim = dq_c;
                }
            }
        }
        F_ext_EE_0_lowpass_prev_ = F_ext_EE_0_lowpass;
        for (size_t i = 0; i < 7; i++) {
            joint_handles_[i].setCommand(dq_c_lim(i));
        }
        
        Eigen::Matrix<double,2,1> u_eigen;
        u_eigen.setZero();
        std::vector<Eigen::VectorXd> custom_data(12);
        custom_data[0] = F_ext_S_s;
        custom_data[1] = F_ext_EE_0;
        custom_data[2] = F_ext_EE_0_lowpass;
        custom_data[3] = dq_c;
        custom_data[4] = dq_c_opt;
        custom_data[5] = u_eigen;
        custom_data[6] = dq_c_lim;
        custom_data[7] = dq_min_safe;
        custom_data[8] = dq_max_safe;
        custom_data[9] = B;
        Eigen::Matrix<double,1,1> limits_violated_vector;
        limits_violated_vector[0] = limits_violated;
        custom_data[10] = limits_violated_vector;
        Eigen::Matrix<double,1,1> locked_vector;
        locked_vector[0] = lockingFunction.locked_;
        custom_data[11] = locked_vector;
        data_extraction_.update_data(&state_handle_,&model_handle_,custom_data);
    }
    
    void TrialControllerVelocity::stopping(const ros::Time&)
    {
        std::vector<std::string> custom_header_values{"F_ext_S_s","F_ext_EE_0","F_ext_EE_0_lowpass","dq_c","dq_c_opt","optimization_vars","dq_c_lim","dq_min_safe","dq_max_safe","nullspace_basis","limits_violated","locked"};
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
    
    void TrialControllerVelocity::trialControllerVelocityParamCallback(
    trial_controller_velocity::trial_controller_velocity_paramConfig& config,
    uint32_t /*level*/) {
        if (!dyn_params_set) {
            config.m_tr = m_tr_;
            config.m_rot = m_rot_;
            config.d_tr = d_tr_;
            config.d_rot = d_rot_;
            dyn_params_set = true;
        }
        else {
            m_tr_ = config.m_tr;
            m_rot_ = config.m_rot;
            d_tr_ = config.d_tr;
            d_rot_ = config.d_rot;
        }
    }
}

PLUGINLIB_EXPORT_CLASS(trial_controller_velocity::TrialControllerVelocity,controller_interface::ControllerBase)
