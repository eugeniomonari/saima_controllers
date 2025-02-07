#include "simple_admittance_controller.h"
#include <pluginlib/class_list_macros.h>
#include <franka/rate_limiting.h>
#include <chrono>
#include <ctime>


namespace simple_admittance_controller
{
    bool SimpleAdmittanceController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle)
    {
        double FT_filter_level = 0;
        if (!node_handle.getParam("FT_filter_level", FT_filter_level)) {
            ROS_ERROR_STREAM("HandGuidanceController: could not get FT_filter_level from parameter server");
            return false;
        }
        std::string eth_interface_name = "";
        if (!node_handle.getParam("eth_interface_name", eth_interface_name)) {
            ROS_ERROR_STREAM("HandGuidanceController: could not get FT_filter_level from parameter server");
            return false;
        }
        initOperations.initFrankaVelFT(robot_hw,&state_handle_,&model_handle_,&joint_handles_,FT_sensor,external_force_computation,FT_filter_level,eth_interface_name, &planning_scene_);
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
        F_ext_EE_0_lowpass_prev_.setZero();
        if (!node_handle.getParam("only_transl", only_transl_)) {
            ROS_ERROR_STREAM("HandGuidanceController: could not get only_transl from parameter server");
            return false;
        }
        if (!node_handle.getParam("only_x", only_x_)) {
            ROS_ERROR_STREAM("HandGuidanceController: could not get only_x from parameter server");
            return false;
        }
        if (!node_handle.getParam("lowpass_filter_cutoff_freq", lowpass_filter_cutoff_freq_)) {
            ROS_ERROR_STREAM("HandGuidanceController: could not get lowpass_filter_cutoff_freq from parameter server");
            return false;
        }
        dynamic_reconfigure_simple_admittance_controller_param_node_ =
        ros::NodeHandle("dynamic_reconfigure_simple_admittance_controller_param_node");
        dynamic_server_simple_admittance_controller_param_ = std::make_unique<dynamic_reconfigure::Server<simple_admittance_controller::simple_admittance_controller_paramConfig>>( dynamic_reconfigure_simple_admittance_controller_param_node_);
        dynamic_server_simple_admittance_controller_param_->setCallback(boost::bind(&SimpleAdmittanceController::trialControllerVelocityParamCallback, this, _1, _2));
        
        return true;
    }
    
    void SimpleAdmittanceController::update(const ros::Time&, const ros::Duration& period)
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
            F_ext_EE_0[3] = 0;
            F_ext_EE_0[4] = 0;
            F_ext_EE_0[5] = 0;
        }
        if (only_x_) {
            F_ext_EE_0[1] = 0;
            F_ext_EE_0[2] = 0;
            F_ext_EE_0[3] = 0;
            F_ext_EE_0[4] = 0;
            F_ext_EE_0[5] = 0;
        }
        
        
        std::array<double, 42> J_vector = model_handle_->getZeroJacobian(franka::Frame::kEndEffector,state_handle_->getRobotState().q_d,state_handle_->getRobotState().F_T_EE,state_handle_->getRobotState().EE_T_K);
        Eigen::Map<Eigen::Matrix<double, 6, 7>> J(J_vector.data());
        Eigen::Matrix<double,7,1> q_next = q_d+dq_d*T_;
        std::array<double,7> q_next_array;
        for (size_t i = 0; i < 7; i++) {
            q_next_array[i] = q_next(i);
        }
        std::array<double, 42> J_next_array = model_handle_->getZeroJacobian(franka::Frame::kEndEffector,q_next_array,state_handle_->getRobotState().F_T_EE,state_handle_->getRobotState().EE_T_K);
            Eigen::Map<Eigen::Matrix<double, 6, 7>> J_next(J_next_array.data());
        std::array<double, 49> B_next_array = model_handle_->getMass(q_next_array,state_handle_->getRobotState().I_total,state_handle_->getRobotState().m_total,state_handle_->getRobotState().F_x_Ctotal);
        Eigen::Matrix<double,7,7> B_next = Eigen::Map<Eigen::Matrix<double, 7, 7>>(B_next_array.data());
        Eigen::Matrix<double,7,6> J_pinv_next = B_next.inverse()*J_next.transpose()*(J_next*B_next.inverse()*J_next.transpose()).inverse();
        Eigen::Matrix<double,2,1> velocity_limits;
        Eigen::Matrix<double,7,1> dq_max_safe;
        Eigen::Matrix<double,7,1> dq_min_safe;
        for (size_t i = 0; i < 7; i++) {
            velocity_limits = setSafeVelocities(kMaxJointPosition[i],kMinJointPosition[i],kMaxJointVelocity[i],kMaxJointAcceleration[i],kMaxJointJerk[i],q_d[i],dq_d[i],ddq_d[i]);
            dq_min_safe[i] = velocity_limits[0];
            dq_max_safe[i] = velocity_limits[1];
        }
        Eigen::Matrix<double,6,1> F_ext_EE_0_lowpass;
        F_ext_EE_0_lowpass.setZero();
        Eigen::Matrix<double,7,1> dq_c = Eigen::Matrix<double,7,1>::Zero();
        Eigen::Matrix<double,7,1> dq_c_lim = Eigen::Matrix<double,7,1>::Zero();
        Eigen::Matrix<double,6,1> v_c = Eigen::Matrix<double,6,1>::Zero();
        bool limits_violated = false;
        for (size_t i=0;i<6;++i){
            F_ext_EE_0_lowpass[i] = franka::lowpassFilter(T_,F_ext_EE_0[i],F_ext_EE_0_lowpass_prev_[i],lowpass_filter_cutoff_freq_);
        }
        if (lockingFunction.locked_) {
            dq_c.setZero();
        }
        else {
            Eigen::Matrix<double,6,1> v_prev = J*dq_d;
            for (size_t i = 0; i < 6; i++) {
                if (i < 3) {
                    v_c[i] = (m_tr_*v_prev[i]+F_ext_EE_0_lowpass(i)*T_)/(m_tr_+d_tr_*T_);
                }
                else {
                    v_c[i] = (m_rot_*v_prev[i]+F_ext_EE_0_lowpass(i)*T_)/(m_rot_+d_rot_*T_);
                }
            }
            dq_c = J_pinv_next*v_c;
        }
        F_ext_EE_0_lowpass_prev_ = F_ext_EE_0_lowpass;
        for (size_t i = 0; i < 7; i++) {
            if (dq_c[i] > dq_max_safe[i] || dq_c[i] < dq_min_safe[i]) {
                limits_violated = true;
            }
            dq_c_lim(i) = std::max(std::min(dq_c[i], dq_max_safe[i]), dq_min_safe[i]);
        }
        
//         robot_state::RobotState& current_state = planning_scene_->getCurrentStateNonConst();
//         const std::vector<std::string> names = current_state.getVariableNames();
//         const double desired_positions[7] = {0,-0.785,0,-2.356,0,1.571,0.785};
//         current_state.setVariablePositions(desired_positions);
//         collision_detection::CollisionRequest collision_request;
//         collision_detection::CollisionResult collision_result;
//         planning_scene_->checkSelfCollision(collision_request, collision_result);
//         ROS_INFO_STREAM("Test 1: Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");
/*        double* positions = current_state.getVariablePositions();
//         const double* poss = current_state.getJointPositions("panda_joint1");
//         std::cout << names[7] << std::endl;
        std::cout << positions[0] << " " << positions[1] << " " << positions[6] << std::endl;*/ 
//         robot_state::RobotState& current_state = planning_scene_.getCurrentStateNonConst();
        collisionFreeCommand.setCommand(q_d,dq_c_lim,&planning_scene_,&joint_handles_);
        
//         for (size_t i = 0; i < 7; i++) {
//             joint_handles_[i].setCommand(dq_c_lim(i));
//         }

        
        std::vector<Eigen::VectorXd> custom_data(15);
        custom_data[0] = F_ext_EE_0;
        custom_data[1] = F_ext_EE_0_lowpass;
        custom_data[2] = dq_max_safe;
        custom_data[3] = dq_min_safe;
        Eigen::Matrix<double,42,1> J_eigen;
        for (size_t i = 0; i < 6; i++) {
            for (size_t j = 0; j < 7; j++) {
                J_eigen(i*7+j) = J(i,j);
            }
        }
        custom_data[4] = J_eigen;
        Eigen::Matrix<double,42,1> J_next_eigen;
        for (size_t i = 0; i < 6; i++) {
            for (size_t j = 0; j < 7; j++) {
                J_next_eigen(i*7+j) = J_next(i,j);
            }
        }
        custom_data[5] = J_next_eigen;
        
        Eigen::Matrix<double,49,1> B_eigen_next;
        for (size_t i = 0; i < 7; i++) {
            for (size_t j = 0; j < 7; j++) {
                B_eigen_next(i*7+j) = B_next(i,j);
            }
        }
        custom_data[6] = B_eigen_next;
        Eigen::Matrix<double,42,1> J_pinv_next_eigen;
        for (size_t i = 0; i < 7; i++) {
            for (size_t j = 0; j < 6; j++) {
                J_pinv_next_eigen(i*6+j) = J_pinv_next(i,j);
            }
        }
        custom_data[7] = J_pinv_next_eigen;
        custom_data[8] = v_c;
        Eigen::Matrix<double,1,1> m_tr_eigen;
        m_tr_eigen[0] = m_tr_;
        custom_data[9] = m_tr_eigen;
        Eigen::Matrix<double,1,1> d_tr_eigen;
        d_tr_eigen[0] = d_tr_;
        custom_data[10] = d_tr_eigen;
        Eigen::Matrix<double,1,1> m_rot_eigen;
        m_rot_eigen[0] = m_rot_;
        custom_data[11] = m_rot_eigen;
        Eigen::Matrix<double,1,1> d_rot_eigen;
        d_rot_eigen[0] = d_rot_;
        custom_data[12] = d_rot_eigen;
        Eigen::Matrix<double,1,1> limits_violated_eigen;
        limits_violated_eigen[0] = limits_violated;
        custom_data[13] = limits_violated_eigen;
        Eigen::Matrix<double,1,1> locked_eigen;
        locked_eigen[0] = lockingFunction.locked_;
        custom_data[14] = locked_eigen;
        data_extraction_.update_data(&state_handle_,&model_handle_,custom_data);
    }
    
    void SimpleAdmittanceController::stopping(const ros::Time&)
    {
        std::vector<std::string> custom_header_values{"F_ext_EE_0","F_ext_EE_0_lowpass","dq_max_safe","dq_min_safe","J","J_next","B_next","J_pinv_next","v_c","m_tr","d_tr","m_rot","d_rot","limits_violated","locked"};
        data_extraction_.write_data_to_csv(custom_header_values);
        data_extraction_.started = false;
    }
    
    Eigen::Matrix<double,2,1> SimpleAdmittanceController::setSafeVelocities(double max_position, double min_position, double max_velocity,double max_acceleration,double max_jerk,double last_commanded_position,double last_commanded_velocity,double last_commanded_acceleration) {
        double safe_max_acceleration = std::max(std::min({
            (max_jerk / max_acceleration) * (max_velocity - last_commanded_velocity), max_acceleration,last_commanded_acceleration+max_jerk*kDeltaT}),last_commanded_acceleration-max_jerk*kDeltaT);
        double safe_min_acceleration = std::min(std::max({
            (max_jerk / max_acceleration) * (-max_velocity - last_commanded_velocity), -max_acceleration,last_commanded_acceleration-max_jerk*kDeltaT}),last_commanded_acceleration+max_jerk*kDeltaT);
        Eigen::Matrix<double,2,1> safe_velocities;
        safe_velocities.setZero();
        safe_velocities[0] = std::min(std::max(last_commanded_velocity+safe_min_acceleration*kDeltaT,(max_acceleration / max_velocity) * (min_position - last_commanded_position)),last_commanded_velocity+safe_max_acceleration*kDeltaT);
        safe_velocities[1] = std::max(std::min(last_commanded_velocity+safe_max_acceleration*kDeltaT,(max_acceleration / max_velocity) * (max_position - last_commanded_position)),last_commanded_velocity+safe_min_acceleration*kDeltaT);
        return safe_velocities;
    }
    
    void SimpleAdmittanceController::trialControllerVelocityParamCallback(
    simple_admittance_controller::simple_admittance_controller_paramConfig& config,
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

PLUGINLIB_EXPORT_CLASS(simple_admittance_controller::SimpleAdmittanceController,controller_interface::ControllerBase)
