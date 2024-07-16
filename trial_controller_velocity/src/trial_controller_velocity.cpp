#include "trial_controller_velocity.h"
#include <Eigen/Dense>
#include <pluginlib/class_list_macros.h>
#include <franka/rate_limiting.h>
// #include <optimizer1_bindings.hpp>

namespace trial_controller_velocity
{
    bool TrialControllerVelocity::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle)
    {
        initOperations.initFrankaVelFT(robot_hw,&state_handle_,&model_handle_,&joint_handles_,FT_sensor,external_force_computation,0);
        F_ext_EE_0_lowpass_prev_.setZero();
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
        bool only_transl = false;
        if (only_transl) {
//             F_ext_EE_0[1] = 0;
//             F_ext_EE_0[2] = 0;
            F_ext_EE_0[3] = 0;
            F_ext_EE_0[4] = 0;
            F_ext_EE_0[5] = 0;
        }
        
        Eigen::Matrix<double,7,1> dq_max_safe;
        Eigen::Matrix<double,7,1> dq_min_safe;
        Eigen::Matrix<double,7,1> B;
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
        Eigen::Matrix<double,7,1> dq_c_opt;
        dq_c_opt.setZero();
        double u[8] = {10, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        qpOASES::real_t solution[2] = {1/mass_[0], 0.0};
//         double u[OPTIMIZER4_NUM_DECISION_VARIABLES] = {mass_[0], 0.0, 0.0, 0,0,0,0,0};
//         double u[OPTIMIZER5_NUM_DECISION_VARIABLES] = {mass_[0], 0.0};
        Eigen::Matrix<double,7,1> dq_c_lim;
        bool limits_violated = false;
        bool only_damping = false;
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
            if (only_damping) {
                Eigen::Matrix<double,6,1> v_c;
                for (size_t i=0;i<6;++i){
                    F_ext_EE_0_lowpass[i] = franka::lowpassFilter(0.001,F_ext_EE_0[i],F_ext_EE_0_lowpass_prev_[i],1);
                }
                F_ext_EE_0_lowpass_prev_ = F_ext_EE_0_lowpass;
                for (size_t i = 0; i < 6; i++) {
                    v_c[i] = F_ext_EE_0_lowpass[i]/damping[i];
                }
                Eigen::Matrix<double,7,6> J_pinv;
                std::array<double, 42> J_vector = model_handle_->getZeroJacobian(franka::Frame::kEndEffector,state_handle_->getRobotState().q_d,state_handle_->getRobotState().F_T_EE,state_handle_->getRobotState().EE_T_K);
                Eigen::Map<Eigen::Matrix<double, 6, 7>> J(J_vector.data());
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
                    Eigen::Matrix<double,7,6> jj = J_pinv;
                    Eigen::Matrix<double,6,1> ff = F_ext_EE_0_lowpass;
                    Eigen::FullPivLU<Eigen::MatrixXd> lu(J);
                    Eigen::MatrixXd J_null = lu.kernel();
                    double p[OPTIMIZER2_NUM_PARAMETERS] = {jj(0,0),jj(0,1),jj(0,2),jj(0,3),jj(0,4),jj(0,5),     jj(1,0),jj(1,1),jj(1,2),jj(1,3),jj(1,4),jj(1,5),        jj(2,0),jj(2,1),jj(2,2),jj(2,3),jj(2,4),jj(2,5),        jj(3,0),jj(3,1),jj(3,2),jj(3,3),jj(3,4),jj(3,5),        jj(4,0),jj(4,1),jj(4,2),jj(4,3),jj(4,4),jj(4,5),        jj(5,0),jj(5,1),jj(5,2),jj(5,3),jj(5,4),jj(5,5),        jj(6,0),jj(6,1),jj(6,2),jj(6,3),jj(6,4),jj(6,5),
                    ff[0], ff[1], ff[2],ff[3], ff[4], ff[5],
                    J_null(0),J_null(1),J_null(2),J_null(3),J_null(4),J_null(5),J_null(6),
                    dq_max_safe[0],dq_max_safe[1],dq_max_safe[2],dq_max_safe[3],dq_max_safe[4],dq_max_safe[5],dq_max_safe[6],
                    dq_min_safe[0],dq_min_safe[1],dq_min_safe[2],dq_min_safe[3],dq_min_safe[4],dq_min_safe[5],dq_min_safe[6],
                    10.0, 0.3, 1000.0, 0.001, 20.0, 10000.0
                    };  // parameter
                    /*double u[OPTIMIZER2_NUM_DECISION_VARIABLES] = {10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};*/  // initial guess
                    
                    optimizer2Cache *cache = optimizer2_new();
                    optimizer2SolverStatus status = optimizer2_solve(cache, u, p, 0, 0);
                    optimizer2_free(cache);
                    Eigen::Matrix<double,6,1> delta;
                    delta[0] = u[2];
                    delta[1] = u[3];
                    delta[2] = u[4];
                    delta[3] = u[5];
                    delta[4] = u[6];
                    delta[5] = u[7];
                    Eigen::Matrix<double,6,1> ff_d;
                    ff_d.head(3) = ff.head(3)/u[0];
                    ff_d.tail(3) = ff.tail(3)/0.3;
                    dq_c_opt = jj*(ff_d+delta)+u[1]*J_null;
                    for (size_t i = 0; i < 7; i++) {
                        dq_c_lim(i) = std::max(std::min(dq_c_opt[i], dq_max_safe[i]), dq_min_safe[i]);
                    }
                }
                else {
                    dq_c_lim = dq_c;
                }
            }
            else {
                Eigen::Matrix<double,6,1> v_c;
                for (size_t i=0;i<6;++i){
                    F_ext_EE_0_lowpass[i] = franka::lowpassFilter(0.001,F_ext_EE_0[i],F_ext_EE_0_lowpass_prev_[i],1);
                }
                F_ext_EE_0_lowpass_prev_ = F_ext_EE_0_lowpass;
                std::array<double, 42> J_vector = model_handle_->getZeroJacobian(franka::Frame::kEndEffector,state_handle_->getRobotState().q_d,state_handle_->getRobotState().F_T_EE,state_handle_->getRobotState().EE_T_K);
                Eigen::Map<Eigen::Matrix<double, 6, 7>> J(J_vector.data());
                Eigen::Matrix<double,6,1> v_prev = J*dq_d;
                for (size_t i = 0; i < 6; i++) {
//                     v_c[i] = 1/(1+damping_[i]/mass_[i]*T_)*(v_prev[i]+F_ext_EE_0_lowpass[i]/mass_[i]*T_);
                    v_c[i] = (1-damping_[i]/mass_[i]*T_)*v_prev[i]+F_ext_EE_0(i)*T_/mass_[i];
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
                    double k1 = 1;
                    double k2 = 1;
                    qpOASES::real_t H[2*2] = { 0,0,
                        0,k2
                    };
                    qpOASES::real_t g[2] = { -k1,0 };
//                     Eigen::DiagonalMatrix<double,8> H;
//                     double k1 = 1;
//                     double k2 = 1;
//                     double k3 = 100;
//                     H.diagonal() << k1,k2,k3,k3,k3,k3,k3,k3;
                    double d_tr = damping[0];
                    double T = T_;
                    Eigen::FullPivLU<Eigen::MatrixXd> lu(J);
                    B = lu.kernel();
                    qpOASES::real_t A[7*2] = {J_pinv(0,0)*(F_ext_EE_0_lowpass(0)*T - T*d_tr*v_prev(0)) + J_pinv(0,1)*(F_ext_EE_0_lowpass(1)*T - T*d_tr*v_prev(1)) + J_pinv(0,2)*(F_ext_EE_0_lowpass(2)*T - T*d_tr*v_prev(2)),  B(0),
                    J_pinv(1,0)*(F_ext_EE_0_lowpass(0)*T - T*d_tr*v_prev(0)) + J_pinv(1,1)*(F_ext_EE_0_lowpass(1)*T - T*d_tr*v_prev(1)) + J_pinv(1,2)*(F_ext_EE_0_lowpass(2)*T - T*d_tr*v_prev(2)),  B(1),
                    J_pinv(2,0)*(F_ext_EE_0_lowpass(0)*T - T*d_tr*v_prev(0)) + J_pinv(2,1)*(F_ext_EE_0_lowpass(1)*T - T*d_tr*v_prev(1)) + J_pinv(2,2)*(F_ext_EE_0_lowpass(2)*T - T*d_tr*v_prev(2)),  B(2),
                    J_pinv(3,0)*(F_ext_EE_0_lowpass(0)*T - T*d_tr*v_prev(0)) + J_pinv(3,1)*(F_ext_EE_0_lowpass(1)*T - T*d_tr*v_prev(1)) + J_pinv(3,2)*(F_ext_EE_0_lowpass(2)*T - T*d_tr*v_prev(2)),  B(3),
                    J_pinv(4,0)*(F_ext_EE_0_lowpass(0)*T - T*d_tr*v_prev(0)) + J_pinv(4,1)*(F_ext_EE_0_lowpass(1)*T - T*d_tr*v_prev(1)) + J_pinv(4,2)*(F_ext_EE_0_lowpass(2)*T - T*d_tr*v_prev(2)),  B(4),
                    J_pinv(5,0)*(F_ext_EE_0_lowpass(0)*T - T*d_tr*v_prev(0)) + J_pinv(5,1)*(F_ext_EE_0_lowpass(1)*T - T*d_tr*v_prev(1)) + J_pinv(5,2)*(F_ext_EE_0_lowpass(2)*T - T*d_tr*v_prev(2)),  B(5),
                    J_pinv(6,0)*(F_ext_EE_0_lowpass(0)*T - T*d_tr*v_prev(0)) + J_pinv(6,1)*(F_ext_EE_0_lowpass(1)*T - T*d_tr*v_prev(1)) + J_pinv(6,2)*(F_ext_EE_0_lowpass(2)*T - T*d_tr*v_prev(2)),  B(6)};
//                     A.setZero();
//                     double d_tr = damping[0];
//                     double T = T_;
//                     A << J_pinv(0,0)*(F_ext_EE_0_lowpass(0)*T - T*d_tr*v_prev(0)) + J_pinv(0,1)*(F_ext_EE_0_lowpass(1)*T - T*d_tr*v_prev(1)) + J_pinv(0,2)*(F_ext_EE_0_lowpass(2)*T - T*d_tr*v_prev(2)),  B(0),  J_pinv(0,0),  J_pinv(0,1),  J_pinv(0,2),  J_pinv(0,3),  J_pinv(0,4),  J_pinv(0,5),
//                     J_pinv(1,0)*(F_ext_EE_0_lowpass(0)*T - T*d_tr*v_prev(0)) + J_pinv(1,1)*(F_ext_EE_0_lowpass(1)*T - T*d_tr*v_prev(1)) + J_pinv(1,2)*(F_ext_EE_0_lowpass(2)*T - T*d_tr*v_prev(2)),  B(1),  J_pinv(1,0),  J_pinv(1,1),  J_pinv(1,2),  J_pinv(1,3),  J_pinv(1,4),  J_pinv(1,5),
//                     J_pinv(2,0)*(F_ext_EE_0_lowpass(0)*T - T*d_tr*v_prev(0)) + J_pinv(2,1)*(F_ext_EE_0_lowpass(1)*T - T*d_tr*v_prev(1)) + J_pinv(2,2)*(F_ext_EE_0_lowpass(2)*T - T*d_tr*v_prev(2)),  B(2),  J_pinv(2,0),  J_pinv(2,1),  J_pinv(2,2),  J_pinv(2,3),  J_pinv(2,4),  J_pinv(2,5),
//                     J_pinv(3,0)*(F_ext_EE_0_lowpass(0)*T - T*d_tr*v_prev(0)) + J_pinv(3,1)*(F_ext_EE_0_lowpass(1)*T - T*d_tr*v_prev(1)) + J_pinv(3,2)*(F_ext_EE_0_lowpass(2)*T - T*d_tr*v_prev(2)),  B(3),  J_pinv(3,0),  J_pinv(3,1),  J_pinv(3,2),  J_pinv(3,3),  J_pinv(3,4),  J_pinv(3,5),
//                     J_pinv(4,0)*(F_ext_EE_0_lowpass(0)*T - T*d_tr*v_prev(0)) + J_pinv(4,1)*(F_ext_EE_0_lowpass(1)*T - T*d_tr*v_prev(1)) + J_pinv(4,2)*(F_ext_EE_0_lowpass(2)*T - T*d_tr*v_prev(2)),  B(4),  J_pinv(4,0),  J_pinv(4,1),  J_pinv(4,2),  J_pinv(4,3),  J_pinv(4,4),  J_pinv(4,5),
//                     J_pinv(5,0)*(F_ext_EE_0_lowpass(0)*T - T*d_tr*v_prev(0)) + J_pinv(5,1)*(F_ext_EE_0_lowpass(1)*T - T*d_tr*v_prev(1)) + J_pinv(5,2)*(F_ext_EE_0_lowpass(2)*T - T*d_tr*v_prev(2)),  B(5),  J_pinv(5,0),  J_pinv(5,1),  J_pinv(5,2),  J_pinv(5,3),  J_pinv(5,4),  J_pinv(5,5),
//                     J_pinv(6,0)*(F_ext_EE_0_lowpass(0)*T - T*d_tr*v_prev(0)) + J_pinv(6,1)*(F_ext_EE_0_lowpass(1)*T - T*d_tr*v_prev(1)) + J_pinv(6,2)*(F_ext_EE_0_lowpass(2)*T - T*d_tr*v_prev(2)),  B(6),  J_pinv(6,0),  J_pinv(6,1),  J_pinv(6,2),  J_pinv(6,3),  J_pinv(6,4),  J_pinv(6,5);
                    
                    double d_rot = damping[3];
                    double m_rot = mass_[3];
                    
                    qpOASES::real_t ubA[7] = {dq_max_safe(0) - J_pinv(0,0)*v_prev(0) - J_pinv(0,1)*v_prev(1) - J_pinv(0,2)*v_prev(2) - J_pinv(0,3)*(v_prev(3) + (F_ext_EE_0_lowpass(3)*T - T*d_rot*v_prev(3))/m_rot) - J_pinv(0,4)*(v_prev(4) + (F_ext_EE_0_lowpass(4)*T - T*d_rot*v_prev(4))/m_rot) - J_pinv(0,5)*(v_prev(5) + (F_ext_EE_0_lowpass(5)*T - T*d_rot*v_prev(5))/m_rot),
                    dq_max_safe(1) - J_pinv(1,0)*v_prev(0) - J_pinv(1,1)*v_prev(1) - J_pinv(1,2)*v_prev(2) - J_pinv(1,3)*(v_prev(3) + (F_ext_EE_0_lowpass(3)*T - T*d_rot*v_prev(3))/m_rot) - J_pinv(1,4)*(v_prev(4) + (F_ext_EE_0_lowpass(4)*T - T*d_rot*v_prev(4))/m_rot) - J_pinv(1,5)*(v_prev(5) + (F_ext_EE_0_lowpass(5)*T - T*d_rot*v_prev(5))/m_rot),
                    dq_max_safe(2) - J_pinv(2,0)*v_prev(0) - J_pinv(2,1)*v_prev(1) - J_pinv(2,2)*v_prev(2) - J_pinv(2,3)*(v_prev(3) + (F_ext_EE_0_lowpass(3)*T - T*d_rot*v_prev(3))/m_rot) - J_pinv(2,4)*(v_prev(4) + (F_ext_EE_0_lowpass(4)*T - T*d_rot*v_prev(4))/m_rot) - J_pinv(2,5)*(v_prev(5) + (F_ext_EE_0_lowpass(5)*T - T*d_rot*v_prev(5))/m_rot),
                    dq_max_safe(3) - J_pinv(3,0)*v_prev(0) - J_pinv(3,1)*v_prev(1) - J_pinv(3,2)*v_prev(2) - J_pinv(3,3)*(v_prev(3) + (F_ext_EE_0_lowpass(3)*T - T*d_rot*v_prev(3))/m_rot) - J_pinv(3,4)*(v_prev(4) + (F_ext_EE_0_lowpass(4)*T - T*d_rot*v_prev(4))/m_rot) - J_pinv(3,5)*(v_prev(5) + (F_ext_EE_0_lowpass(5)*T - T*d_rot*v_prev(5))/m_rot),
                    dq_max_safe(4) - J_pinv(4,0)*v_prev(0) - J_pinv(4,1)*v_prev(1) - J_pinv(4,2)*v_prev(2) - J_pinv(4,3)*(v_prev(3) + (F_ext_EE_0_lowpass(3)*T - T*d_rot*v_prev(3))/m_rot) - J_pinv(4,4)*(v_prev(4) + (F_ext_EE_0_lowpass(4)*T - T*d_rot*v_prev(4))/m_rot) - J_pinv(4,5)*(v_prev(5) + (F_ext_EE_0_lowpass(5)*T - T*d_rot*v_prev(5))/m_rot),
                    dq_max_safe(5) - J_pinv(5,0)*v_prev(0) - J_pinv(5,1)*v_prev(1) - J_pinv(5,2)*v_prev(2) - J_pinv(5,3)*(v_prev(3) + (F_ext_EE_0_lowpass(3)*T - T*d_rot*v_prev(3))/m_rot) - J_pinv(5,4)*(v_prev(4) + (F_ext_EE_0_lowpass(4)*T - T*d_rot*v_prev(4))/m_rot) - J_pinv(5,5)*(v_prev(5) + (F_ext_EE_0_lowpass(5)*T - T*d_rot*v_prev(5))/m_rot),
                    dq_max_safe(6) - J_pinv(6,0)*v_prev(0) - J_pinv(6,1)*v_prev(1) - J_pinv(6,2)*v_prev(2) - J_pinv(6,3)*(v_prev(3) + (F_ext_EE_0_lowpass(3)*T - T*d_rot*v_prev(3))/m_rot) - J_pinv(6,4)*(v_prev(4) + (F_ext_EE_0_lowpass(4)*T - T*d_rot*v_prev(4))/m_rot) - J_pinv(6,5)*(v_prev(5) + (F_ext_EE_0_lowpass(5)*T - T*d_rot*v_prev(5))/m_rot)};
//                     ubA.setZero();
//                     ubA << dq_max_safe(0) - J_pinv(0,0)*v_prev(0) - J_pinv(0,1)*v_prev(1) - J_pinv(0,2)*v_prev(2) - J_pinv(0,3)*(v_prev(3) + (F_ext_EE_0_lowpass(3)*T - T*d_rot*v_prev(3))/m_rot) - J_pinv(0,4)*(v_prev(4) + (F_ext_EE_0_lowpass(4)*T - T*d_rot*v_prev(4))/m_rot) - J_pinv(0,5)*(v_prev(5) + (F_ext_EE_0_lowpass(5)*T - T*d_rot*v_prev(5))/m_rot),
//                     dq_max_safe(1) - J_pinv(1,0)*v_prev(0) - J_pinv(1,1)*v_prev(1) - J_pinv(1,2)*v_prev(2) - J_pinv(1,3)*(v_prev(3) + (F_ext_EE_0_lowpass(3)*T - T*d_rot*v_prev(3))/m_rot) - J_pinv(1,4)*(v_prev(4) + (F_ext_EE_0_lowpass(4)*T - T*d_rot*v_prev(4))/m_rot) - J_pinv(1,5)*(v_prev(5) + (F_ext_EE_0_lowpass(5)*T - T*d_rot*v_prev(5))/m_rot),
//                     dq_max_safe(2) - J_pinv(2,0)*v_prev(0) - J_pinv(2,1)*v_prev(1) - J_pinv(2,2)*v_prev(2) - J_pinv(2,3)*(v_prev(3) + (F_ext_EE_0_lowpass(3)*T - T*d_rot*v_prev(3))/m_rot) - J_pinv(2,4)*(v_prev(4) + (F_ext_EE_0_lowpass(4)*T - T*d_rot*v_prev(4))/m_rot) - J_pinv(2,5)*(v_prev(5) + (F_ext_EE_0_lowpass(5)*T - T*d_rot*v_prev(5))/m_rot),
//                     dq_max_safe(3) - J_pinv(3,0)*v_prev(0) - J_pinv(3,1)*v_prev(1) - J_pinv(3,2)*v_prev(2) - J_pinv(3,3)*(v_prev(3) + (F_ext_EE_0_lowpass(3)*T - T*d_rot*v_prev(3))/m_rot) - J_pinv(3,4)*(v_prev(4) + (F_ext_EE_0_lowpass(4)*T - T*d_rot*v_prev(4))/m_rot) - J_pinv(3,5)*(v_prev(5) + (F_ext_EE_0_lowpass(5)*T - T*d_rot*v_prev(5))/m_rot),
//                     dq_max_safe(4) - J_pinv(4,0)*v_prev(0) - J_pinv(4,1)*v_prev(1) - J_pinv(4,2)*v_prev(2) - J_pinv(4,3)*(v_prev(3) + (F_ext_EE_0_lowpass(3)*T - T*d_rot*v_prev(3))/m_rot) - J_pinv(4,4)*(v_prev(4) + (F_ext_EE_0_lowpass(4)*T - T*d_rot*v_prev(4))/m_rot) - J_pinv(4,5)*(v_prev(5) + (F_ext_EE_0_lowpass(5)*T - T*d_rot*v_prev(5))/m_rot),
//                     dq_max_safe(5) - J_pinv(5,0)*v_prev(0) - J_pinv(5,1)*v_prev(1) - J_pinv(5,2)*v_prev(2) - J_pinv(5,3)*(v_prev(3) + (F_ext_EE_0_lowpass(3)*T - T*d_rot*v_prev(3))/m_rot) - J_pinv(5,4)*(v_prev(4) + (F_ext_EE_0_lowpass(4)*T - T*d_rot*v_prev(4))/m_rot) - J_pinv(5,5)*(v_prev(5) + (F_ext_EE_0_lowpass(5)*T - T*d_rot*v_prev(5))/m_rot),
//                     dq_max_safe(6) - J_pinv(6,0)*v_prev(0) - J_pinv(6,1)*v_prev(1) - J_pinv(6,2)*v_prev(2) - J_pinv(6,3)*(v_prev(3) + (F_ext_EE_0_lowpass(3)*T - T*d_rot*v_prev(3))/m_rot) - J_pinv(6,4)*(v_prev(4) + (F_ext_EE_0_lowpass(4)*T - T*d_rot*v_prev(4))/m_rot) - J_pinv(6,5)*(v_prev(5) + (F_ext_EE_0_lowpass(5)*T - T*d_rot*v_prev(5))/m_rot);
                    
                    qpOASES::real_t lbA[7] = {dq_min_safe(0) - J_pinv(0,0)*v_prev(0) - J_pinv(0,1)*v_prev(1) - J_pinv(0,2)*v_prev(2) - J_pinv(0,3)*(v_prev(3) + (F_ext_EE_0_lowpass(3)*T - T*d_rot*v_prev(3))/m_rot) - J_pinv(0,4)*(v_prev(4) + (F_ext_EE_0_lowpass(4)*T - T*d_rot*v_prev(4))/m_rot) - J_pinv(0,5)*(v_prev(5) + (F_ext_EE_0_lowpass(5)*T - T*d_rot*v_prev(5))/m_rot),
                    dq_min_safe(1) - J_pinv(1,0)*v_prev(0) - J_pinv(1,1)*v_prev(1) - J_pinv(1,2)*v_prev(2) - J_pinv(1,3)*(v_prev(3) + (F_ext_EE_0_lowpass(3)*T - T*d_rot*v_prev(3))/m_rot) - J_pinv(1,4)*(v_prev(4) + (F_ext_EE_0_lowpass(4)*T - T*d_rot*v_prev(4))/m_rot) - J_pinv(1,5)*(v_prev(5) + (F_ext_EE_0_lowpass(5)*T - T*d_rot*v_prev(5))/m_rot),
                    dq_min_safe(2) - J_pinv(2,0)*v_prev(0) - J_pinv(2,1)*v_prev(1) - J_pinv(2,2)*v_prev(2) - J_pinv(2,3)*(v_prev(3) + (F_ext_EE_0_lowpass(3)*T - T*d_rot*v_prev(3))/m_rot) - J_pinv(2,4)*(v_prev(4) + (F_ext_EE_0_lowpass(4)*T - T*d_rot*v_prev(4))/m_rot) - J_pinv(2,5)*(v_prev(5) + (F_ext_EE_0_lowpass(5)*T - T*d_rot*v_prev(5))/m_rot),
                    dq_min_safe(3) - J_pinv(3,0)*v_prev(0) - J_pinv(3,1)*v_prev(1) - J_pinv(3,2)*v_prev(2) - J_pinv(3,3)*(v_prev(3) + (F_ext_EE_0_lowpass(3)*T - T*d_rot*v_prev(3))/m_rot) - J_pinv(3,4)*(v_prev(4) + (F_ext_EE_0_lowpass(4)*T - T*d_rot*v_prev(4))/m_rot) - J_pinv(3,5)*(v_prev(5) + (F_ext_EE_0_lowpass(5)*T - T*d_rot*v_prev(5))/m_rot),
                    dq_min_safe(4) - J_pinv(4,0)*v_prev(0) - J_pinv(4,1)*v_prev(1) - J_pinv(4,2)*v_prev(2) - J_pinv(4,3)*(v_prev(3) + (F_ext_EE_0_lowpass(3)*T - T*d_rot*v_prev(3))/m_rot) - J_pinv(4,4)*(v_prev(4) + (F_ext_EE_0_lowpass(4)*T - T*d_rot*v_prev(4))/m_rot) - J_pinv(4,5)*(v_prev(5) + (F_ext_EE_0_lowpass(5)*T - T*d_rot*v_prev(5))/m_rot),
                    dq_min_safe(5) - J_pinv(5,0)*v_prev(0) - J_pinv(5,1)*v_prev(1) - J_pinv(5,2)*v_prev(2) - J_pinv(5,3)*(v_prev(3) + (F_ext_EE_0_lowpass(3)*T - T*d_rot*v_prev(3))/m_rot) - J_pinv(5,4)*(v_prev(4) + (F_ext_EE_0_lowpass(4)*T - T*d_rot*v_prev(4))/m_rot) - J_pinv(5,5)*(v_prev(5) + (F_ext_EE_0_lowpass(5)*T - T*d_rot*v_prev(5))/m_rot),
                    dq_min_safe(6) - J_pinv(6,0)*v_prev(0) - J_pinv(6,1)*v_prev(1) - J_pinv(6,2)*v_prev(2) - J_pinv(6,3)*(v_prev(3) + (F_ext_EE_0_lowpass(3)*T - T*d_rot*v_prev(3))/m_rot) - J_pinv(6,4)*(v_prev(4) + (F_ext_EE_0_lowpass(4)*T - T*d_rot*v_prev(4))/m_rot) - J_pinv(6,5)*(v_prev(5) + (F_ext_EE_0_lowpass(5)*T - T*d_rot*v_prev(5))/m_rot)};
//                     lbA.setZero();
//                     lbA << dq_min_safe(0) - J_pinv(0,0)*v_prev(0) - J_pinv(0,1)*v_prev(1) - J_pinv(0,2)*v_prev(2) - J_pinv(0,3)*(v_prev(3) + (F_ext_EE_0_lowpass(3)*T - T*d_rot*v_prev(3))/m_rot) - J_pinv(0,4)*(v_prev(4) + (F_ext_EE_0_lowpass(4)*T - T*d_rot*v_prev(4))/m_rot) - J_pinv(0,5)*(v_prev(5) + (F_ext_EE_0_lowpass(5)*T - T*d_rot*v_prev(5))/m_rot),
//                     dq_min_safe(1) - J_pinv(1,0)*v_prev(0) - J_pinv(1,1)*v_prev(1) - J_pinv(1,2)*v_prev(2) - J_pinv(1,3)*(v_prev(3) + (F_ext_EE_0_lowpass(3)*T - T*d_rot*v_prev(3))/m_rot) - J_pinv(1,4)*(v_prev(4) + (F_ext_EE_0_lowpass(4)*T - T*d_rot*v_prev(4))/m_rot) - J_pinv(1,5)*(v_prev(5) + (F_ext_EE_0_lowpass(5)*T - T*d_rot*v_prev(5))/m_rot),
//                     dq_min_safe(2) - J_pinv(2,0)*v_prev(0) - J_pinv(2,1)*v_prev(1) - J_pinv(2,2)*v_prev(2) - J_pinv(2,3)*(v_prev(3) + (F_ext_EE_0_lowpass(3)*T - T*d_rot*v_prev(3))/m_rot) - J_pinv(2,4)*(v_prev(4) + (F_ext_EE_0_lowpass(4)*T - T*d_rot*v_prev(4))/m_rot) - J_pinv(2,5)*(v_prev(5) + (F_ext_EE_0_lowpass(5)*T - T*d_rot*v_prev(5))/m_rot),
//                     dq_min_safe(3) - J_pinv(3,0)*v_prev(0) - J_pinv(3,1)*v_prev(1) - J_pinv(3,2)*v_prev(2) - J_pinv(3,3)*(v_prev(3) + (F_ext_EE_0_lowpass(3)*T - T*d_rot*v_prev(3))/m_rot) - J_pinv(3,4)*(v_prev(4) + (F_ext_EE_0_lowpass(4)*T - T*d_rot*v_prev(4))/m_rot) - J_pinv(3,5)*(v_prev(5) + (F_ext_EE_0_lowpass(5)*T - T*d_rot*v_prev(5))/m_rot),
//                     dq_min_safe(4) - J_pinv(4,0)*v_prev(0) - J_pinv(4,1)*v_prev(1) - J_pinv(4,2)*v_prev(2) - J_pinv(4,3)*(v_prev(3) + (F_ext_EE_0_lowpass(3)*T - T*d_rot*v_prev(3))/m_rot) - J_pinv(4,4)*(v_prev(4) + (F_ext_EE_0_lowpass(4)*T - T*d_rot*v_prev(4))/m_rot) - J_pinv(4,5)*(v_prev(5) + (F_ext_EE_0_lowpass(5)*T - T*d_rot*v_prev(5))/m_rot),
//                     dq_min_safe(5) - J_pinv(5,0)*v_prev(0) - J_pinv(5,1)*v_prev(1) - J_pinv(5,2)*v_prev(2) - J_pinv(5,3)*(v_prev(3) + (F_ext_EE_0_lowpass(3)*T - T*d_rot*v_prev(3))/m_rot) - J_pinv(5,4)*(v_prev(4) + (F_ext_EE_0_lowpass(4)*T - T*d_rot*v_prev(4))/m_rot) - J_pinv(5,5)*(v_prev(5) + (F_ext_EE_0_lowpass(5)*T - T*d_rot*v_prev(5))/m_rot),
//                     dq_min_safe(6) - J_pinv(6,0)*v_prev(0) - J_pinv(6,1)*v_prev(1) - J_pinv(6,2)*v_prev(2) - J_pinv(6,3)*(v_prev(3) + (F_ext_EE_0_lowpass(3)*T - T*d_rot*v_prev(3))/m_rot) - J_pinv(6,4)*(v_prev(4) + (F_ext_EE_0_lowpass(4)*T - T*d_rot*v_prev(4))/m_rot) - J_pinv(6,5)*(v_prev(5) + (F_ext_EE_0_lowpass(5)*T - T*d_rot*v_prev(5))/m_rot);
                    
                    qpOASES::real_t lb[2] = {1/20,-qpOASES::INFTY};
                    qpOASES::real_t ub[2] = {1/mass_[0],qpOASES::INFTY};
                    
                    int nWSR = 100;
                    qpOASES::SQProblem opt_problem( 2,7 );
//                     qpOASES::returnValue return_value = opt_problem.init( H,g,A,lb,ub,lbA,ubA, nWSR,0,solution );
// //                     std::cout << qpOASES::getSimpleStatus(return_value) << std::endl;
//                     
//                     
//                     opt_problem.getPrimalSolution(solution);
//                     if (robot_state.current_errors) {
//                         error = true;
//                     }
//                         
//                     if (!error) {
//                         std::cout << qpOASES::getSimpleStatus(return_value) << std::endl << 1/solution[0] << std::endl << solution[1] << std::endl << std::endl;
//                     }
//                     
//                     double m_tr = 1/solution[0];
// //                     std::cout << m_tr << std::endl;
//                     double a = solution[1];
//                     Eigen::Matrix<double,6,1> v_c_lim;
//                     v_c_lim.setZero();
//                     v_c_lim.head(3) = (1-d_tr/m_tr*T_)*v_prev.head(3)+F_ext_EE_0_lowpass.head(3)*T_/m_tr;
//                     v_c_lim.tail(3) = (1-d_rot/m_rot*T_)*v_prev.tail(3)+F_ext_EE_0_lowpass.tail(3)*T_/m_rot;
//                     dq_c_lim = J_pinv*(v_c_lim)+B*a;
                    
                    
                    
//                     double k1 = 1;
//                     double k2 = 1;
//                     double k3 = 10000; 
//                     qpOASES::real_t H[8*8] = { 0,0,0,0,0,0,0,0,
//                         0,k2,0,0,0,0,0,0,
//                         0,0,k3,0,0,0,0,0,
//                         0,0,0,k3,0,0,0,0,
//                         0,0,0,0,k3,0,0,0,
//                         0,0,0,0,0,k3,0,0,
//                         0,0,0,0,0,0,k3,0,
//                         0,0,0,0,0,0,0,k3
//                     };
//                     qpOASES::real_t g[8] = { -k1,0,0,0,0,0,0,0 };
// //                     Eigen::DiagonalMatrix<double,8> H;
// //                     double k1 = 1;
// //                     double k2 = 1;
// //                     double k3 = 100;
// //                     H.diagonal() << k1,k2,k3,k3,k3,k3,k3,k3;
//                     double d_tr = damping[0];
//                     double T = T_;
//                     Eigen::FullPivLU<Eigen::MatrixXd> lu(J);
//                     Eigen::MatrixXd B = lu.kernel();
//                     qpOASES::real_t A[7*8] = {J_pinv(0,0)*(F_ext_EE_0_lowpass(0)*T - T*d_tr*v_prev(0)) + J_pinv(0,1)*(F_ext_EE_0_lowpass(1)*T - T*d_tr*v_prev(1)) + J_pinv(0,2)*(F_ext_EE_0_lowpass(2)*T - T*d_tr*v_prev(2)),  B(0),  J_pinv(0,0),  J_pinv(0,1),  J_pinv(0,2),  J_pinv(0,3),  J_pinv(0,4),  J_pinv(0,5),
//                     J_pinv(1,0)*(F_ext_EE_0_lowpass(0)*T - T*d_tr*v_prev(0)) + J_pinv(1,1)*(F_ext_EE_0_lowpass(1)*T - T*d_tr*v_prev(1)) + J_pinv(1,2)*(F_ext_EE_0_lowpass(2)*T - T*d_tr*v_prev(2)),  B(1),  J_pinv(1,0),  J_pinv(1,1),  J_pinv(1,2),  J_pinv(1,3),  J_pinv(1,4),  J_pinv(1,5),
//                     J_pinv(2,0)*(F_ext_EE_0_lowpass(0)*T - T*d_tr*v_prev(0)) + J_pinv(2,1)*(F_ext_EE_0_lowpass(1)*T - T*d_tr*v_prev(1)) + J_pinv(2,2)*(F_ext_EE_0_lowpass(2)*T - T*d_tr*v_prev(2)),  B(2),  J_pinv(2,0),  J_pinv(2,1),  J_pinv(2,2),  J_pinv(2,3),  J_pinv(2,4),  J_pinv(2,5),
//                     J_pinv(3,0)*(F_ext_EE_0_lowpass(0)*T - T*d_tr*v_prev(0)) + J_pinv(3,1)*(F_ext_EE_0_lowpass(1)*T - T*d_tr*v_prev(1)) + J_pinv(3,2)*(F_ext_EE_0_lowpass(2)*T - T*d_tr*v_prev(2)),  B(3),  J_pinv(3,0),  J_pinv(3,1),  J_pinv(3,2),  J_pinv(3,3),  J_pinv(3,4),  J_pinv(3,5),
//                     J_pinv(4,0)*(F_ext_EE_0_lowpass(0)*T - T*d_tr*v_prev(0)) + J_pinv(4,1)*(F_ext_EE_0_lowpass(1)*T - T*d_tr*v_prev(1)) + J_pinv(4,2)*(F_ext_EE_0_lowpass(2)*T - T*d_tr*v_prev(2)),  B(4),  J_pinv(4,0),  J_pinv(4,1),  J_pinv(4,2),  J_pinv(4,3),  J_pinv(4,4),  J_pinv(4,5),
//                     J_pinv(5,0)*(F_ext_EE_0_lowpass(0)*T - T*d_tr*v_prev(0)) + J_pinv(5,1)*(F_ext_EE_0_lowpass(1)*T - T*d_tr*v_prev(1)) + J_pinv(5,2)*(F_ext_EE_0_lowpass(2)*T - T*d_tr*v_prev(2)),  B(5),  J_pinv(5,0),  J_pinv(5,1),  J_pinv(5,2),  J_pinv(5,3),  J_pinv(5,4),  J_pinv(5,5),
//                     J_pinv(6,0)*(F_ext_EE_0_lowpass(0)*T - T*d_tr*v_prev(0)) + J_pinv(6,1)*(F_ext_EE_0_lowpass(1)*T - T*d_tr*v_prev(1)) + J_pinv(6,2)*(F_ext_EE_0_lowpass(2)*T - T*d_tr*v_prev(2)),  B(6),  J_pinv(6,0),  J_pinv(6,1),  J_pinv(6,2),  J_pinv(6,3),  J_pinv(6,4),  J_pinv(6,5)};
// //                     A.setZero();
// //                     double d_tr = damping[0];
// //                     double T = T_;
// //                     A << J_pinv(0,0)*(F_ext_EE_0_lowpass(0)*T - T*d_tr*v_prev(0)) + J_pinv(0,1)*(F_ext_EE_0_lowpass(1)*T - T*d_tr*v_prev(1)) + J_pinv(0,2)*(F_ext_EE_0_lowpass(2)*T - T*d_tr*v_prev(2)),  B(0),  J_pinv(0,0),  J_pinv(0,1),  J_pinv(0,2),  J_pinv(0,3),  J_pinv(0,4),  J_pinv(0,5),
// //                     J_pinv(1,0)*(F_ext_EE_0_lowpass(0)*T - T*d_tr*v_prev(0)) + J_pinv(1,1)*(F_ext_EE_0_lowpass(1)*T - T*d_tr*v_prev(1)) + J_pinv(1,2)*(F_ext_EE_0_lowpass(2)*T - T*d_tr*v_prev(2)),  B(1),  J_pinv(1,0),  J_pinv(1,1),  J_pinv(1,2),  J_pinv(1,3),  J_pinv(1,4),  J_pinv(1,5),
// //                     J_pinv(2,0)*(F_ext_EE_0_lowpass(0)*T - T*d_tr*v_prev(0)) + J_pinv(2,1)*(F_ext_EE_0_lowpass(1)*T - T*d_tr*v_prev(1)) + J_pinv(2,2)*(F_ext_EE_0_lowpass(2)*T - T*d_tr*v_prev(2)),  B(2),  J_pinv(2,0),  J_pinv(2,1),  J_pinv(2,2),  J_pinv(2,3),  J_pinv(2,4),  J_pinv(2,5),
// //                     J_pinv(3,0)*(F_ext_EE_0_lowpass(0)*T - T*d_tr*v_prev(0)) + J_pinv(3,1)*(F_ext_EE_0_lowpass(1)*T - T*d_tr*v_prev(1)) + J_pinv(3,2)*(F_ext_EE_0_lowpass(2)*T - T*d_tr*v_prev(2)),  B(3),  J_pinv(3,0),  J_pinv(3,1),  J_pinv(3,2),  J_pinv(3,3),  J_pinv(3,4),  J_pinv(3,5),
// //                     J_pinv(4,0)*(F_ext_EE_0_lowpass(0)*T - T*d_tr*v_prev(0)) + J_pinv(4,1)*(F_ext_EE_0_lowpass(1)*T - T*d_tr*v_prev(1)) + J_pinv(4,2)*(F_ext_EE_0_lowpass(2)*T - T*d_tr*v_prev(2)),  B(4),  J_pinv(4,0),  J_pinv(4,1),  J_pinv(4,2),  J_pinv(4,3),  J_pinv(4,4),  J_pinv(4,5),
// //                     J_pinv(5,0)*(F_ext_EE_0_lowpass(0)*T - T*d_tr*v_prev(0)) + J_pinv(5,1)*(F_ext_EE_0_lowpass(1)*T - T*d_tr*v_prev(1)) + J_pinv(5,2)*(F_ext_EE_0_lowpass(2)*T - T*d_tr*v_prev(2)),  B(5),  J_pinv(5,0),  J_pinv(5,1),  J_pinv(5,2),  J_pinv(5,3),  J_pinv(5,4),  J_pinv(5,5),
// //                     J_pinv(6,0)*(F_ext_EE_0_lowpass(0)*T - T*d_tr*v_prev(0)) + J_pinv(6,1)*(F_ext_EE_0_lowpass(1)*T - T*d_tr*v_prev(1)) + J_pinv(6,2)*(F_ext_EE_0_lowpass(2)*T - T*d_tr*v_prev(2)),  B(6),  J_pinv(6,0),  J_pinv(6,1),  J_pinv(6,2),  J_pinv(6,3),  J_pinv(6,4),  J_pinv(6,5);
//                     
//                     double d_rot = damping[3];
//                     double m_rot = mass_[3];
//                     
//                     qpOASES::real_t ubA[7] = {dq_max_safe(0) - J_pinv(0,0)*v_prev(0) - J_pinv(0,1)*v_prev(1) - J_pinv(0,2)*v_prev(2) - J_pinv(0,3)*(v_prev(3) + (F_ext_EE_0_lowpass(3)*T - T*d_rot*v_prev(3))/m_rot) - J_pinv(0,4)*(v_prev(4) + (F_ext_EE_0_lowpass(4)*T - T*d_rot*v_prev(4))/m_rot) - J_pinv(0,5)*(v_prev(5) + (F_ext_EE_0_lowpass(5)*T - T*d_rot*v_prev(5))/m_rot),
//                     dq_max_safe(1) - J_pinv(1,0)*v_prev(0) - J_pinv(1,1)*v_prev(1) - J_pinv(1,2)*v_prev(2) - J_pinv(1,3)*(v_prev(3) + (F_ext_EE_0_lowpass(3)*T - T*d_rot*v_prev(3))/m_rot) - J_pinv(1,4)*(v_prev(4) + (F_ext_EE_0_lowpass(4)*T - T*d_rot*v_prev(4))/m_rot) - J_pinv(1,5)*(v_prev(5) + (F_ext_EE_0_lowpass(5)*T - T*d_rot*v_prev(5))/m_rot),
//                     dq_max_safe(2) - J_pinv(2,0)*v_prev(0) - J_pinv(2,1)*v_prev(1) - J_pinv(2,2)*v_prev(2) - J_pinv(2,3)*(v_prev(3) + (F_ext_EE_0_lowpass(3)*T - T*d_rot*v_prev(3))/m_rot) - J_pinv(2,4)*(v_prev(4) + (F_ext_EE_0_lowpass(4)*T - T*d_rot*v_prev(4))/m_rot) - J_pinv(2,5)*(v_prev(5) + (F_ext_EE_0_lowpass(5)*T - T*d_rot*v_prev(5))/m_rot),
//                     dq_max_safe(3) - J_pinv(3,0)*v_prev(0) - J_pinv(3,1)*v_prev(1) - J_pinv(3,2)*v_prev(2) - J_pinv(3,3)*(v_prev(3) + (F_ext_EE_0_lowpass(3)*T - T*d_rot*v_prev(3))/m_rot) - J_pinv(3,4)*(v_prev(4) + (F_ext_EE_0_lowpass(4)*T - T*d_rot*v_prev(4))/m_rot) - J_pinv(3,5)*(v_prev(5) + (F_ext_EE_0_lowpass(5)*T - T*d_rot*v_prev(5))/m_rot),
//                     dq_max_safe(4) - J_pinv(4,0)*v_prev(0) - J_pinv(4,1)*v_prev(1) - J_pinv(4,2)*v_prev(2) - J_pinv(4,3)*(v_prev(3) + (F_ext_EE_0_lowpass(3)*T - T*d_rot*v_prev(3))/m_rot) - J_pinv(4,4)*(v_prev(4) + (F_ext_EE_0_lowpass(4)*T - T*d_rot*v_prev(4))/m_rot) - J_pinv(4,5)*(v_prev(5) + (F_ext_EE_0_lowpass(5)*T - T*d_rot*v_prev(5))/m_rot),
//                     dq_max_safe(5) - J_pinv(5,0)*v_prev(0) - J_pinv(5,1)*v_prev(1) - J_pinv(5,2)*v_prev(2) - J_pinv(5,3)*(v_prev(3) + (F_ext_EE_0_lowpass(3)*T - T*d_rot*v_prev(3))/m_rot) - J_pinv(5,4)*(v_prev(4) + (F_ext_EE_0_lowpass(4)*T - T*d_rot*v_prev(4))/m_rot) - J_pinv(5,5)*(v_prev(5) + (F_ext_EE_0_lowpass(5)*T - T*d_rot*v_prev(5))/m_rot),
//                     dq_max_safe(6) - J_pinv(6,0)*v_prev(0) - J_pinv(6,1)*v_prev(1) - J_pinv(6,2)*v_prev(2) - J_pinv(6,3)*(v_prev(3) + (F_ext_EE_0_lowpass(3)*T - T*d_rot*v_prev(3))/m_rot) - J_pinv(6,4)*(v_prev(4) + (F_ext_EE_0_lowpass(4)*T - T*d_rot*v_prev(4))/m_rot) - J_pinv(6,5)*(v_prev(5) + (F_ext_EE_0_lowpass(5)*T - T*d_rot*v_prev(5))/m_rot)};
// //                     ubA.setZero();
// //                     ubA << dq_max_safe(0) - J_pinv(0,0)*v_prev(0) - J_pinv(0,1)*v_prev(1) - J_pinv(0,2)*v_prev(2) - J_pinv(0,3)*(v_prev(3) + (F_ext_EE_0_lowpass(3)*T - T*d_rot*v_prev(3))/m_rot) - J_pinv(0,4)*(v_prev(4) + (F_ext_EE_0_lowpass(4)*T - T*d_rot*v_prev(4))/m_rot) - J_pinv(0,5)*(v_prev(5) + (F_ext_EE_0_lowpass(5)*T - T*d_rot*v_prev(5))/m_rot),
// //                     dq_max_safe(1) - J_pinv(1,0)*v_prev(0) - J_pinv(1,1)*v_prev(1) - J_pinv(1,2)*v_prev(2) - J_pinv(1,3)*(v_prev(3) + (F_ext_EE_0_lowpass(3)*T - T*d_rot*v_prev(3))/m_rot) - J_pinv(1,4)*(v_prev(4) + (F_ext_EE_0_lowpass(4)*T - T*d_rot*v_prev(4))/m_rot) - J_pinv(1,5)*(v_prev(5) + (F_ext_EE_0_lowpass(5)*T - T*d_rot*v_prev(5))/m_rot),
// //                     dq_max_safe(2) - J_pinv(2,0)*v_prev(0) - J_pinv(2,1)*v_prev(1) - J_pinv(2,2)*v_prev(2) - J_pinv(2,3)*(v_prev(3) + (F_ext_EE_0_lowpass(3)*T - T*d_rot*v_prev(3))/m_rot) - J_pinv(2,4)*(v_prev(4) + (F_ext_EE_0_lowpass(4)*T - T*d_rot*v_prev(4))/m_rot) - J_pinv(2,5)*(v_prev(5) + (F_ext_EE_0_lowpass(5)*T - T*d_rot*v_prev(5))/m_rot),
// //                     dq_max_safe(3) - J_pinv(3,0)*v_prev(0) - J_pinv(3,1)*v_prev(1) - J_pinv(3,2)*v_prev(2) - J_pinv(3,3)*(v_prev(3) + (F_ext_EE_0_lowpass(3)*T - T*d_rot*v_prev(3))/m_rot) - J_pinv(3,4)*(v_prev(4) + (F_ext_EE_0_lowpass(4)*T - T*d_rot*v_prev(4))/m_rot) - J_pinv(3,5)*(v_prev(5) + (F_ext_EE_0_lowpass(5)*T - T*d_rot*v_prev(5))/m_rot),
// //                     dq_max_safe(4) - J_pinv(4,0)*v_prev(0) - J_pinv(4,1)*v_prev(1) - J_pinv(4,2)*v_prev(2) - J_pinv(4,3)*(v_prev(3) + (F_ext_EE_0_lowpass(3)*T - T*d_rot*v_prev(3))/m_rot) - J_pinv(4,4)*(v_prev(4) + (F_ext_EE_0_lowpass(4)*T - T*d_rot*v_prev(4))/m_rot) - J_pinv(4,5)*(v_prev(5) + (F_ext_EE_0_lowpass(5)*T - T*d_rot*v_prev(5))/m_rot),
// //                     dq_max_safe(5) - J_pinv(5,0)*v_prev(0) - J_pinv(5,1)*v_prev(1) - J_pinv(5,2)*v_prev(2) - J_pinv(5,3)*(v_prev(3) + (F_ext_EE_0_lowpass(3)*T - T*d_rot*v_prev(3))/m_rot) - J_pinv(5,4)*(v_prev(4) + (F_ext_EE_0_lowpass(4)*T - T*d_rot*v_prev(4))/m_rot) - J_pinv(5,5)*(v_prev(5) + (F_ext_EE_0_lowpass(5)*T - T*d_rot*v_prev(5))/m_rot),
// //                     dq_max_safe(6) - J_pinv(6,0)*v_prev(0) - J_pinv(6,1)*v_prev(1) - J_pinv(6,2)*v_prev(2) - J_pinv(6,3)*(v_prev(3) + (F_ext_EE_0_lowpass(3)*T - T*d_rot*v_prev(3))/m_rot) - J_pinv(6,4)*(v_prev(4) + (F_ext_EE_0_lowpass(4)*T - T*d_rot*v_prev(4))/m_rot) - J_pinv(6,5)*(v_prev(5) + (F_ext_EE_0_lowpass(5)*T - T*d_rot*v_prev(5))/m_rot);
//                     
//                     qpOASES::real_t lbA[7] = {dq_min_safe(0) - J_pinv(0,0)*v_prev(0) - J_pinv(0,1)*v_prev(1) - J_pinv(0,2)*v_prev(2) - J_pinv(0,3)*(v_prev(3) + (F_ext_EE_0_lowpass(3)*T - T*d_rot*v_prev(3))/m_rot) - J_pinv(0,4)*(v_prev(4) + (F_ext_EE_0_lowpass(4)*T - T*d_rot*v_prev(4))/m_rot) - J_pinv(0,5)*(v_prev(5) + (F_ext_EE_0_lowpass(5)*T - T*d_rot*v_prev(5))/m_rot),
//                     dq_min_safe(1) - J_pinv(1,0)*v_prev(0) - J_pinv(1,1)*v_prev(1) - J_pinv(1,2)*v_prev(2) - J_pinv(1,3)*(v_prev(3) + (F_ext_EE_0_lowpass(3)*T - T*d_rot*v_prev(3))/m_rot) - J_pinv(1,4)*(v_prev(4) + (F_ext_EE_0_lowpass(4)*T - T*d_rot*v_prev(4))/m_rot) - J_pinv(1,5)*(v_prev(5) + (F_ext_EE_0_lowpass(5)*T - T*d_rot*v_prev(5))/m_rot),
//                     dq_min_safe(2) - J_pinv(2,0)*v_prev(0) - J_pinv(2,1)*v_prev(1) - J_pinv(2,2)*v_prev(2) - J_pinv(2,3)*(v_prev(3) + (F_ext_EE_0_lowpass(3)*T - T*d_rot*v_prev(3))/m_rot) - J_pinv(2,4)*(v_prev(4) + (F_ext_EE_0_lowpass(4)*T - T*d_rot*v_prev(4))/m_rot) - J_pinv(2,5)*(v_prev(5) + (F_ext_EE_0_lowpass(5)*T - T*d_rot*v_prev(5))/m_rot),
//                     dq_min_safe(3) - J_pinv(3,0)*v_prev(0) - J_pinv(3,1)*v_prev(1) - J_pinv(3,2)*v_prev(2) - J_pinv(3,3)*(v_prev(3) + (F_ext_EE_0_lowpass(3)*T - T*d_rot*v_prev(3))/m_rot) - J_pinv(3,4)*(v_prev(4) + (F_ext_EE_0_lowpass(4)*T - T*d_rot*v_prev(4))/m_rot) - J_pinv(3,5)*(v_prev(5) + (F_ext_EE_0_lowpass(5)*T - T*d_rot*v_prev(5))/m_rot),
//                     dq_min_safe(4) - J_pinv(4,0)*v_prev(0) - J_pinv(4,1)*v_prev(1) - J_pinv(4,2)*v_prev(2) - J_pinv(4,3)*(v_prev(3) + (F_ext_EE_0_lowpass(3)*T - T*d_rot*v_prev(3))/m_rot) - J_pinv(4,4)*(v_prev(4) + (F_ext_EE_0_lowpass(4)*T - T*d_rot*v_prev(4))/m_rot) - J_pinv(4,5)*(v_prev(5) + (F_ext_EE_0_lowpass(5)*T - T*d_rot*v_prev(5))/m_rot),
//                     dq_min_safe(5) - J_pinv(5,0)*v_prev(0) - J_pinv(5,1)*v_prev(1) - J_pinv(5,2)*v_prev(2) - J_pinv(5,3)*(v_prev(3) + (F_ext_EE_0_lowpass(3)*T - T*d_rot*v_prev(3))/m_rot) - J_pinv(5,4)*(v_prev(4) + (F_ext_EE_0_lowpass(4)*T - T*d_rot*v_prev(4))/m_rot) - J_pinv(5,5)*(v_prev(5) + (F_ext_EE_0_lowpass(5)*T - T*d_rot*v_prev(5))/m_rot),
//                     dq_min_safe(6) - J_pinv(6,0)*v_prev(0) - J_pinv(6,1)*v_prev(1) - J_pinv(6,2)*v_prev(2) - J_pinv(6,3)*(v_prev(3) + (F_ext_EE_0_lowpass(3)*T - T*d_rot*v_prev(3))/m_rot) - J_pinv(6,4)*(v_prev(4) + (F_ext_EE_0_lowpass(4)*T - T*d_rot*v_prev(4))/m_rot) - J_pinv(6,5)*(v_prev(5) + (F_ext_EE_0_lowpass(5)*T - T*d_rot*v_prev(5))/m_rot)};
// //                     lbA.setZero();
// //                     lbA << dq_min_safe(0) - J_pinv(0,0)*v_prev(0) - J_pinv(0,1)*v_prev(1) - J_pinv(0,2)*v_prev(2) - J_pinv(0,3)*(v_prev(3) + (F_ext_EE_0_lowpass(3)*T - T*d_rot*v_prev(3))/m_rot) - J_pinv(0,4)*(v_prev(4) + (F_ext_EE_0_lowpass(4)*T - T*d_rot*v_prev(4))/m_rot) - J_pinv(0,5)*(v_prev(5) + (F_ext_EE_0_lowpass(5)*T - T*d_rot*v_prev(5))/m_rot),
// //                     dq_min_safe(1) - J_pinv(1,0)*v_prev(0) - J_pinv(1,1)*v_prev(1) - J_pinv(1,2)*v_prev(2) - J_pinv(1,3)*(v_prev(3) + (F_ext_EE_0_lowpass(3)*T - T*d_rot*v_prev(3))/m_rot) - J_pinv(1,4)*(v_prev(4) + (F_ext_EE_0_lowpass(4)*T - T*d_rot*v_prev(4))/m_rot) - J_pinv(1,5)*(v_prev(5) + (F_ext_EE_0_lowpass(5)*T - T*d_rot*v_prev(5))/m_rot),
// //                     dq_min_safe(2) - J_pinv(2,0)*v_prev(0) - J_pinv(2,1)*v_prev(1) - J_pinv(2,2)*v_prev(2) - J_pinv(2,3)*(v_prev(3) + (F_ext_EE_0_lowpass(3)*T - T*d_rot*v_prev(3))/m_rot) - J_pinv(2,4)*(v_prev(4) + (F_ext_EE_0_lowpass(4)*T - T*d_rot*v_prev(4))/m_rot) - J_pinv(2,5)*(v_prev(5) + (F_ext_EE_0_lowpass(5)*T - T*d_rot*v_prev(5))/m_rot),
// //                     dq_min_safe(3) - J_pinv(3,0)*v_prev(0) - J_pinv(3,1)*v_prev(1) - J_pinv(3,2)*v_prev(2) - J_pinv(3,3)*(v_prev(3) + (F_ext_EE_0_lowpass(3)*T - T*d_rot*v_prev(3))/m_rot) - J_pinv(3,4)*(v_prev(4) + (F_ext_EE_0_lowpass(4)*T - T*d_rot*v_prev(4))/m_rot) - J_pinv(3,5)*(v_prev(5) + (F_ext_EE_0_lowpass(5)*T - T*d_rot*v_prev(5))/m_rot),
// //                     dq_min_safe(4) - J_pinv(4,0)*v_prev(0) - J_pinv(4,1)*v_prev(1) - J_pinv(4,2)*v_prev(2) - J_pinv(4,3)*(v_prev(3) + (F_ext_EE_0_lowpass(3)*T - T*d_rot*v_prev(3))/m_rot) - J_pinv(4,4)*(v_prev(4) + (F_ext_EE_0_lowpass(4)*T - T*d_rot*v_prev(4))/m_rot) - J_pinv(4,5)*(v_prev(5) + (F_ext_EE_0_lowpass(5)*T - T*d_rot*v_prev(5))/m_rot),
// //                     dq_min_safe(5) - J_pinv(5,0)*v_prev(0) - J_pinv(5,1)*v_prev(1) - J_pinv(5,2)*v_prev(2) - J_pinv(5,3)*(v_prev(3) + (F_ext_EE_0_lowpass(3)*T - T*d_rot*v_prev(3))/m_rot) - J_pinv(5,4)*(v_prev(4) + (F_ext_EE_0_lowpass(4)*T - T*d_rot*v_prev(4))/m_rot) - J_pinv(5,5)*(v_prev(5) + (F_ext_EE_0_lowpass(5)*T - T*d_rot*v_prev(5))/m_rot),
// //                     dq_min_safe(6) - J_pinv(6,0)*v_prev(0) - J_pinv(6,1)*v_prev(1) - J_pinv(6,2)*v_prev(2) - J_pinv(6,3)*(v_prev(3) + (F_ext_EE_0_lowpass(3)*T - T*d_rot*v_prev(3))/m_rot) - J_pinv(6,4)*(v_prev(4) + (F_ext_EE_0_lowpass(4)*T - T*d_rot*v_prev(4))/m_rot) - J_pinv(6,5)*(v_prev(5) + (F_ext_EE_0_lowpass(5)*T - T*d_rot*v_prev(5))/m_rot);
//                     
//                     qpOASES::real_t lb[8] = {1/20,-qpOASES::INFTY,-qpOASES::INFTY,-qpOASES::INFTY,-qpOASES::INFTY,-qpOASES::INFTY,-qpOASES::INFTY,-qpOASES::INFTY};
//                     qpOASES::real_t ub[8] = {1/mass_[0],qpOASES::INFTY,qpOASES::INFTY,qpOASES::INFTY,qpOASES::INFTY,qpOASES::INFTY,qpOASES::INFTY,qpOASES::INFTY};
//                     
//                     int nWSR = 100;
//                     qpOASES::SQProblem opt_problem( 8,7 );
//                     qpOASES::returnValue return_value = opt_problem.init( H,g,A,lb,ub,lbA,ubA, nWSR,0,solution );
//                     std::cout << qpOASES::getSimpleStatus(return_value) << std::endl;
//                     
//                     
//                     opt_problem.getPrimalSolution(solution);
//                     for (size_t i = 0; i < 8; i++) {
//                         if (i==0) {
//                             std::cout << 1/solution[i] << std::endl;
//                         }
//                         else {
//                             std::cout << solution[i] << std::endl;
//                         }
//                     }
//                     
//                     double m_tr = 1/solution[0];
// //                     std::cout << m_tr << std::endl;
//                     double a = solution[1];
//                     Eigen::Matrix<double,6,1> delta;
//                     delta.setZero();
//                     delta << solution[2],solution[3],solution[4],solution[5],solution[6],solution[7];
//                     Eigen::Matrix<double,6,1> v_c_lim;
//                     v_c_lim.setZero();
//                     v_c_lim.head(3) = (1-d_tr/m_tr*T_)*v_prev.head(3)+F_ext_EE_0_lowpass.head(3)*T_/m_tr;
//                     v_c_lim.tail(3) = (1-d_rot/m_rot*T_)*v_prev.tail(3)+F_ext_EE_0_lowpass.tail(3)*T_/m_rot;
//                     dq_c_lim = J_pinv*(v_c_lim+delta)+B*a;
                    
                    
//                     Eigen::Matrix<double,7,6> jj = J_pinv;
//                     Eigen::Matrix<double,6,1> ff = F_ext_EE_0_lowpass;
//                     Eigen::FullPivLU<Eigen::MatrixXd> lu(J);
//                     Eigen::MatrixXd J_null = lu.kernel();
//                     double p[OPTIMIZER4_NUM_PARAMETERS] = {jj(0,0),jj(0,1),jj(0,2),jj(0,3),jj(0,4),jj(0,5),     jj(1,0),jj(1,1),jj(1,2),jj(1,3),jj(1,4),jj(1,5),        jj(2,0),jj(2,1),jj(2,2),jj(2,3),jj(2,4),jj(2,5),        jj(3,0),jj(3,1),jj(3,2),jj(3,3),jj(3,4),jj(3,5),        jj(4,0),jj(4,1),jj(4,2),jj(4,3),jj(4,4),jj(4,5),        jj(5,0),jj(5,1),jj(5,2),jj(5,3),jj(5,4),jj(5,5),        jj(6,0),jj(6,1),jj(6,2),jj(6,3),jj(6,4),jj(6,5),
//                     ff[0], ff[1], ff[2],ff[3], ff[4], ff[5],
//                     v_prev[0], v_prev[1], v_prev[2],v_prev[3],v_prev[4],v_prev[5],
//                     J_null(0),J_null(1),J_null(2),J_null(3),J_null(4),J_null(5),J_null(6),
//                     dq_max_safe[0],dq_max_safe[1],dq_max_safe[2],dq_max_safe[3],dq_max_safe[4],dq_max_safe[5],dq_max_safe[6],
//                     dq_min_safe[0],dq_min_safe[1],dq_min_safe[2],dq_min_safe[3],dq_min_safe[4],dq_min_safe[5],dq_min_safe[6],
//                     mass_[0],mass_[3], damping_[0], damping_[3], 1000.0, 0.001, 20.0, 10000.0
//                     };  // parameter
//                     /*double u[OPTIMIZER2_NUM_DECISION_VARIABLES] = {10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};*/  // initial guess
//                     
//                     optimizer4Cache *cache = optimizer4_new();
//                     optimizer4SolverStatus status = optimizer4_solve(cache, u, p, 0, 0);
//                     std::cout << bool(status.exit_status) << std::endl;
//                     optimizer4_free(cache);
//                     Eigen::Matrix<double,6,1> delta;
//                     delta[0] = u[2];
//                     delta[1] = u[3];
//                     delta[2] = u[4];
//                     delta[3] = u[5];
//                     delta[4] = u[6];
//                     delta[5] = u[7];
//                     Eigen::Matrix<double,6,1> v_c_opt;
//                     v_c_opt.setZero();
//                     for (size_t i = 0; i < 6; i++) {
//                         if (i < 3) {
//                             v_c_opt[i] = 1/(1+damping_[i]/u[0]*T_)*(v_prev[i]+F_ext_EE_0_lowpass[i]/u[0]*T_);
//                         }
//                         else {
//                             v_c_opt[i] = 1/(1+damping_[i]/mass_[i]*T_)*(v_prev[i]+F_ext_EE_0_lowpass[i]/mass_[i]*T_);
//                         }
//                         
//     //                     v_c[i] = (1-damping_[i]/mass_[i]*T_)*v_prev[i]+F_ext_EE_0(i)*T_/mass_[i];
//                     }
//                     dq_c_opt = jj*(v_c_opt+delta)+u[1]*J_null;
// //                     dq_c_opt = jj*(v_c_opt)+u[1]*J_null;
                    for (size_t i = 0; i < 7; i++) {
                        dq_c_lim(i) = std::max(std::min(dq_c[i], dq_max_safe[i]), dq_min_safe[i]);
                    }
                }
                else {
                    dq_c_lim = dq_c;
                }
            }
        }
        
        for (size_t i = 0; i < 7; i++) {
            joint_handles_[i].setCommand(dq_c_lim(i));
        }
        
        Eigen::Matrix<double,2,1> u_eigen;
        for (size_t i = 0; i < 2; i++) {
            u_eigen[i] = solution[i];
        }
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
}

PLUGINLIB_EXPORT_CLASS(trial_controller_velocity::TrialControllerVelocity,controller_interface::ControllerBase)
