#include "trial_controller_velocity.h"
// #include <Eigen/Dense>
#include <pluginlib/class_list_macros.h>
#include <franka/rate_limiting.h>
#include <chrono>
#include <ctime>

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
            initOperations.initFrankaVelFT(robot_hw,&state_handle_,&model_handle_,&joint_handles_,FT_sensor,external_force_computation,7,"enxa0cec88b0292");
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
            initOperations.initFrankaVelFT(robot_hw,&state_handle_,&model_handle_,&joint_handles_,FT_sensor,external_force_computation,0,"enxa0cec88b0292");
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
        
        if (!node_handle.getParam("only_x", only_x_)) {
            ROS_ERROR_STREAM("HandGuidanceController: could not get only_x from parameter server");
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
        Eigen::Map<Eigen::Matrix<double, 7, 1>> q_d(robot_state.q_d.data());
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
        Eigen::Matrix<double,7,7> B_mat = Eigen::Matrix<double,7,7>::Zero();
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
        qpOASES::real_t solution[2] = {m_tr_,d_tr_};
        qpOASES::real_t solution_delta[6] = {0,0,0,0,0,0};
        Eigen::Matrix<double,7,1> dq_c_lim = Eigen::Matrix<double,7,1>::Zero();
        bool limits_violated = false;
        double optimization_result = 99;
        double optimization_result_delta = 99;
        for (size_t i=0;i<6;++i){
            F_ext_EE_0_lowpass[i] = franka::lowpassFilter(0.001,F_ext_EE_0[i],F_ext_EE_0_lowpass_prev_[i],1000);
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
                Eigen::Matrix<double,7,1> q_next = q_d+dq_d*T_;
                std::array<double,7> q_next_array;
                for (size_t i = 0; i < 7; i++) {
                    q_next_array[i] = q_next(i);
                }
                std::array<double, 42> J_vector_next = model_handle_->getZeroJacobian(franka::Frame::kEndEffector,q_next_array,state_handle_->getRobotState().F_T_EE,state_handle_->getRobotState().EE_T_K);
                Eigen::Map<Eigen::Matrix<double, 6, 7>> J_next(J_vector_next.data());
                Eigen::Matrix<double,6,1> v_prev = J*dq_d;
                for (size_t i = 0; i < 6; i++) {
                    if (i < 3) {
                        v_c[i] = (m_tr_*v_prev[i]+F_ext_EE_0_lowpass(i)*T_)/(m_tr_+d_tr_*T_);
                    }
                    else {
                        v_c[i] = (m_rot_*v_prev[i]+F_ext_EE_0_lowpass(i)*T_)/(m_rot_+d_rot_*T_);
                    }
                }
//                 mass (const std::array< double, 7 > &q, const std::array< double, 9 > &I_total, double m_total, const std::array< double, 3 > &F_x_Ctotal)
                std::array<double, 49> B_vector = model_handle_->getMass(q_next_array,state_handle_->getRobotState().I_total,state_handle_->getRobotState().m_total,state_handle_->getRobotState().F_x_Ctotal);
                B_mat = Eigen::Map<Eigen::Matrix<double, 7, 7>>(B_vector.data());
                Eigen::Matrix<double,7,6> J_pinv = B_mat.inverse()*J_next.transpose()*(J_next*B_mat.inverse()*J_next.transpose()).inverse();
//                 Eigen::Matrix<double,7,6> J_pinv = J_next.transpose()*(J_next*J_next.transpose()).inverse();
                dq_c = J_pinv*v_c;
                for (size_t i = 0; i < 7; i++) {
                    if (!(dq_c[i] <= dq_max_safe[i] && dq_c[i] >= dq_min_safe[i])) {
                        limits_violated = true;
                    }
                }
                if (limits_violated) {
                    auto start = std::chrono::system_clock::now();
                    double k1 = 1;
                    double k2 = 1;
                    qpOASES::real_t H[2*2];
                    for (size_t i = 0; i < 2; i++) {
                        for (size_t j = 0; j < 2; j++) {
                            if (i==j) {
                                if (i == 0) {
                                    H[i*2+j] = 2*k1;
                                }
                                else if (i == 1) {
                                    H[i*2+j] = 2*k2;
                                }
                            }
                            else {
                                H[i*2+j] = 0;
                            }
                        }
                    }
                    qpOASES::real_t g[2] = {-2*m_tr_,-2*d_tr_};
                    Eigen::Matrix<double,14,2> A_eigen = Eigen::Matrix<double,14,2>::Zero();
                    Eigen::Matrix<double,7,1> B = dq_max_safe-J_pinv.block(0,3,7,3)*(m_rot_*v_prev.tail(3)+F_ext_EE_0_lowpass.tail(3)*T_)/(m_rot_+d_rot_*T_);
                    Eigen::Matrix<double,7,1> C = dq_min_safe-J_pinv.block(0,3,7,3)*(m_rot_*v_prev.tail(3)+F_ext_EE_0_lowpass.tail(3)*T_)/(m_rot_+d_rot_*T_);
//                     Eigen::Matrix<double,6,1> v_adm_lim = Eigen::Matrix<double,6,1>::Zero();
//                     v_adm_lim.head(3) = (solution[0]*v_prev.head(3)+F_ext_EE_0_lowpass.head(3)*T_)/(solution[0]+solution[1]*T_);
//                     v_adm_lim.tail(3) = (m_rot_*v_prev.tail(3)+F_ext_EE_0_lowpass.tail(3)*T_)/(m_rot_+d_rot_*T_);
//                     dq_c_lim = J_pinv*(v_adm_lim);,7,3)*(m_rot_*v_prev.tail(3)+F_ext_EE_0_lowpass.tail(3)*T_)/(m_rot_+d_rot_*T_);
                    A_eigen.block(0,0,7,1) = J_pinv.block(0,0,7,3)*v_prev.head(3)-B;
                    A_eigen.block(7,0,7,1) = -J_pinv.block(0,0,7,3)*v_prev.head(3)+C;
                    A_eigen.block(0,1,7,1) = -B*T_;
                    A_eigen.block(7,1,7,1) = C*T_;
                    qpOASES::real_t A[14*2];
                    for (size_t i = 0; i < 14; i++) {
                        for (size_t j = 0; j < 2; j++) {
                            A[i*2+j] = A_eigen(i,j);
                        }
                    }
                    Eigen::Matrix<double,14,1> ubA_eigen = Eigen::Matrix<double,14,1>::Zero();
                    ubA_eigen.block(0,0,7,1) = -J_pinv.block(0,0,7,3)*F_ext_EE_0_lowpass.head(3)*T_;
                    ubA_eigen.block(7,0,7,1) = J_pinv.block(0,0,7,3)*F_ext_EE_0_lowpass.head(3)*T_;
                    qpOASES::real_t ubA[14];
                    qpOASES::real_t lbA[14];
                    for (size_t i = 0; i < 14; i++) {
                        ubA[i] = ubA_eigen[i];
                        lbA[i] = -qpOASES::INFTY;
                    }
                    int nWSR = 100;
                    qpOASES::real_t lb[2] = {m_tr_,d_tr_};
                    qpOASES::real_t ub[2] = {qpOASES::INFTY,qpOASES::INFTY};
                    qpOASES::SQProblem opt_problem( 2,14 );
                    qpOASES::returnValue return_value = opt_problem.init( H,g,A,lb,ub,lbA,ubA, nWSR,0,solution );
                    auto end = std::chrono::system_clock::now();
                    std::chrono::duration<double> elapsed_time = end-start;
                    std::cout << elapsed_time.count() << std::endl;
                    opt_problem.getPrimalSolution(solution);
                    optimization_result = qpOASES::getSimpleStatus(return_value);
                    Eigen::Matrix<double,6,1> v_adm_lim = Eigen::Matrix<double,6,1>::Zero();
                    v_adm_lim.head(3) = (solution[0]*v_prev.head(3)+F_ext_EE_0_lowpass.head(3)*T_)/(solution[0]+solution[1]*T_);
                    v_adm_lim.tail(3) = (m_rot_*v_prev.tail(3)+F_ext_EE_0_lowpass.tail(3)*T_)/(m_rot_+d_rot_*T_);
                    dq_c_lim = J_pinv*(v_adm_lim);
//                     if (optimization_result == 0) {
//                         v_adm_lim.head(3) = (solution[0]*v_prev.head(3)+F_ext_EE_0_lowpass.head(3)*T_)/(solution[0]+solution[1]*T_);
//                         v_adm_lim.tail(3) = (m_rot_*v_prev.tail(3)+F_ext_EE_0_lowpass.tail(3)*T_)/(m_rot_+d_rot_*T_);
//                         dq_c_lim = J_pinv*(v_adm_lim);
//                     }
//                     else {
//                         std::cout << "Optimization without delta has failed." << std::endl;
//                         double k_delta = 1;
//                         qpOASES::real_t H_delta[6*6];
//                         for (size_t i = 0; i < 6; i++) {
//                             for (size_t j = 0; j < 6; j++) {
//                                 if (i==j) {
//                                     H_delta[i*2+j] = 2*k_delta;
//                                 }
//                                 else {
//                                     H_delta[i*2+j] = 0;
//                                 }
//                             }
//                         }
//                         qpOASES::real_t g_delta[6] = {0,0,0,0,0,0};
// //                         Eigen::Matrix<double,14,2> A_eigen_delta = Eigen::Matrix<double,14,2>::Zero();
// //                         Eigen::Matrix<double,7,1> B_delta = dq_max_safe-J_pinv.block(0,3,7,3)*F_ext_EE_0_lowpass.tail(3)/d_rot_;
// //                         Eigen::Matrix<double,7,1> C_delta = dq_min_safe-J_pinv.block(0,3,7,3)*F_ext_EE_0_lowpass.tail(3)/d_rot_;
// //                         A_eigen_delta.block(0,0,7,1) = J_pinv.block(0,0,7,3)*v_prev.head(3)-B_delta;
// //                         A_eigen_delta.block(7,0,7,1) = -J_pinv.block(0,0,7,3)*v_prev.head(3)+C_delta;
// //                         A_eigen_delta.block(0,1,7,1) = -B_delta*T_;
// //                         A_eigen_delta.block(7,1,7,1) = C_delta*T_;
//                         qpOASES::real_t A_delta[7*6];
//                         for (size_t i = 0; i < 7; i++) {
//                             for (size_t j = 0; j < 6; j++) {
//                                 A_delta[i*6+j] = J_pinv(i,j);igen::Matrix<double,6,1> v_adm_lim = Eigen::Matrix<double,6,1>::Zero();
//                     v_adm_lim.head(3) = (solution[0]*v_prev.head(3)+F_ext_EE_0_lowpass.head(3)*T_)/(solution[0]+solution[1]*T_);
//                     v_adm_lim.tail(3) = (m_rot_*v_prev.tail(3)+F_ext_EE_0_lowpass.tail(3)*T_)/(m_rot_+d_rot_*T_);
//                     dq_c_lim = J_pinv*(v_adm_lim);
//                             }
//                         }
//                         Eigen::Matrix<double,7,1> ubA_eigen_delta = dq_max_safe-J_pinv.block(0,0,7,3)*(m_tr_*v_prev.head(3)+F_ext_EE_0_lowpass.head(3)*T_)/(m_tr_+d_tr_*T_)-J_pinv.block(0,3,7,3)*(m_rot_*v_prev.tail(3)+F_ext_EE_0_lowpass.tail(3)*T_)/(m_rot_+d_rot_*T_);
//                         Eigen::Matrix<double,7,1> lbA_eigen_delta = dq_min_safe-J_pinv.block(0,0,7,3)*(m_tr_*v_prev.head(3)+F_ext_EE_0_lowpass.head(3)*T_)/(m_tr_+d_tr_*T_)-J_pinv.block(0,3,7,3)*(m_rot_*v_prev.tail(3)+F_ext_EE_0_lowpass.tail(3)*T_)/(m_rot_+d_rot_*T_);
//                         qpOASES::real_t ubA_delta[7];
//                         qpOASES::real_t lbA_delta[7];
//                         for (size_t i = 0; i < 7; i++) {
//                             ubA_delta[i] = ubA_eigen_delta[i];
//                             lbA_delta[i] = lbA_eigen_delta[i];
//                         }
//                         int nWSR_delta = 100;
// //                         qpOASES::real_t lb_delta[2] = {m_tr_,d_tr_}; time
// //                         qpOASES::real_t ub_delta[2] = {qpOASES::INFTY,qpOASES::INFTY};
//                         qpOASES::SQProblem opt_problem_delta( 6,7 );
//                         qpOASES::returnValue return_value_delta = opt_problem_delta.init( H_delta,g_delta,A_delta,NULL,NULL,lbA_delta,ubA_delta, nWSR_delta,0,solution_delta );             
//                         opt_problem_delta.getPrimalSolution(solution_delta);
//                         Eigen::Matrix<double,6,1> delta = Eigen::Matrix<double,6,1>::Zero();
//                         for (size_t i = 0; i < 6; i++) {
//                             delta(i) = solution_delta[i];
//                         }
//                         optimization_result_delta = qpOASES::getSimpleStatus(return_value_delta);
//                         v_adm_lim.head(3) = (m_tr_*v_prev.head(3)+F_ext_EE_0_lowpass.head(3)*T_)/(m_tr_+d_tr_*T_)+delta.head(3);
//                         v_adm_lim.tail(3) = (m_rot_*v_prev.tail(3)+F_ext_EE_0_lowpass.tail(3)*T_)/(m_rot_+d_rot_*T_)+delta.tail(3);
//                         dq_c_lim = J_pinv*(v_adm_lim);
//                     }
                    
//                     double lambda_0 = 1;
//                     qpOASES::real_t H[1*1] = {2};
//                     qpOASES::real_t g[1] = {-2*lambda_0};
//                     Eigen::Matrix<double,7,1> A_eigen = Eigen::Matrix<double,7,1>::Zero();
//                     A_eigen = J_pinv.block(0,0,7,3)*v_c.head(3);
//                     qpOASES::real_t A[7*1];
//                     for (size_t i = 0; i < 7; i++) {
//                         A[i] = A_eigen(i);
//                     }
//                     Eigen::Matrix<double,7,1> ubA_eigen = Eigen::Matrix<double,7,1>::Zero();
//                     ubA_eigen = dq_max_safe - J_pinv.block(0,3,7,3)*v_c.tail(3);
//                     qpOASES::real_t ubA[7*1];
//                     for (size_t i = 0; i < 7; i++) {
//                         ubA[i] = ubA_eigen(i);
//                     }
//                     Eigen::Matrix<double,7,1> lbA_eigen = Eigen::Matrix<double,7,1>::Zero();
//                     lbA_eigen = dq_min_safe - J_pinv.block(0,3,7,3)*v_c.tail(3);
//                     qpOASES::real_t lbA[7*1];
//                     for (size_t i = 0; i < 7; i++) {
//                         lbA[i] = lbA_eigen(i);
//                     }
//                     qpOASES::real_t ub[1] = {lambda_0};
//                     qpOASES::SQProblem opt_problem(1,7);
//                     int nWSR = 100;
//                     qpOASES::returnValue return_value = opt_problem.init(H,g,A,NULL,ub,lbA,ubA,nWSR,NULL);
//                     opt_problem.getPrimalSolution(solution);
//                     Eigen::Matrix<double,6,1> v_adm_lim = Eigen::Matrix<double,6,1>::Zero();
//                     v_adm_lim.head(3) = solution[0]*v_c.head(3);
//                     v_adm_lim.tail(3) = v_c.tail(3);
//                     dq_c_lim = J_pinv*(v_adm_lim);
//                     optimization_result = qpOASES::getSimpleStatus(return_value);
                    
//                     for (size_t i = 0; i < 7; i++) {
//                         dq_c_lim(i) = std::max(std::min(dq_c[i], dq_max_safe[i]), dq_min_safe[i]);
//                     }
                    
                    for (size_t i = 0; i < 7; i++) {
                        dq_c_lim(i) = std::max(std::min(dq_c[i], dq_max_safe[i]), dq_min_safe[i]);
                    }
                }
                else {
                    dq_c_lim = dq_c;
                }
            }
        }
        F_ext_EE_0_lowpass_prev_ = F_ext_EE_0_lowpass;
        for (size_t i = 0; i < 7; i++) {
            joint_handles_[i].setCommand(dq_c_lim(i));
//             joint_handles_[i].setCommand(0);
        }
        
        Eigen::Matrix<double,2,1> u_eigen;
        u_eigen.setZero();
        std::vector<Eigen::VectorXd> custom_data(11);
        custom_data[0] = F_ext_EE_0_lowpass;
        custom_data[1] = dq_c_lim;
        custom_data[2] = dq_max_safe;
        custom_data[3] = dq_min_safe;
        Eigen::Matrix<double,49,1> B_next_vector;
        for (size_t i = 0; i < 7; i++) {
            for (size_t j = 0; j < 7; j++) {
                B_next_vector(i*7+j) = B_mat(i,j);
            }
        }
        custom_data[4] = B_next_vector;
        Eigen::Matrix<double,2,1> optimization_vars_vector;
        optimization_vars_vector[0] = solution[0];
        optimization_vars_vector[1] = solution[1];
        custom_data[5] = optimization_vars_vector;
        Eigen::Matrix<double,6,1> optimization_vars_delta_vector;
        optimization_vars_delta_vector[0] = solution_delta[0];
        optimization_vars_delta_vector[1] = solution_delta[1];
        optimization_vars_delta_vector[2] = solution_delta[2];
        optimization_vars_delta_vector[3] = solution_delta[3];
        optimization_vars_delta_vector[4] = solution_delta[4];
        optimization_vars_delta_vector[5] = solution_delta[5];
        custom_data[6] = optimization_vars_delta_vector;
        Eigen::Matrix<double,1,1> optimization_result_vector;
        optimization_result_vector[0] = optimization_result;
        custom_data[7] = optimization_result_vector;
        Eigen::Matrix<double,1,1> optimization_result_delta_vector;
        optimization_result_delta_vector[0] = optimization_result_delta;
        custom_data[8] = optimization_result_delta_vector;
        Eigen::Matrix<double,1,1> limits_violated_vector;
        limits_violated_vector[0] = limits_violated;
        custom_data[9] = limits_violated_vector;
        Eigen::Matrix<double,1,1> locked_vector;
        locked_vector[0] = lockingFunction.locked_;
        custom_data[10] = locked_vector;
        data_extraction_.update_data(&state_handle_,&model_handle_,custom_data);
    }
    
    void TrialControllerVelocity::stopping(const ros::Time&)
    {
        std::vector<std::string> custom_header_values{"F_ext_EE_0_lowpass","dq_commanded","dq_max_safe","dq_min_safe","B_next","optimization_vars","optimization_vars_delta","optimization_result","optimization_result_delta","limits_violated","locked"};
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
