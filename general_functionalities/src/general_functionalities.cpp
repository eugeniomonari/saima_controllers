#include <general_functionalities.h>
#include <iostream>

namespace general_functionalities
{
    Eigen::Matrix<double, 6, 1> EEPoleBaseFrameExtWrenchComputation::computeEEPoleBaseFrameExtWrench(const Eigen::Matrix<double, 4, 4>& O_T_EE, const Eigen::Matrix<double, 6, 1>& sensor_data, bool bias_checked, bool bias_error, bool ecat_error) {
        double mass_calib = 0.288;
        double max_gravity_torque = 0.045;
        Eigen::Matrix<double,3,1> r_s = Eigen::Vector3d(0,0,-0.088);
        Eigen::Matrix<double, 6, 1> F_ext_S_s;
        Eigen::Vector3d gravity_force_0(0, 0, -mass_calib * 9.81);
        Eigen::Vector3d gravity_force_s_initial = s_R_0_initial_ * gravity_force_0;
        Eigen::Matrix<double, 3, 3> EE_R_O = O_T_EE.block(0, 0, 3, 3).transpose();
        Eigen::Matrix3d s_R_0 = s_R_EE_ * EE_R_O;
        Eigen::Vector3d gravity_force_s = s_R_0 * gravity_force_0;
        F_ext_S_s.head(3) = sensor_data.head(3) + gravity_force_s_initial - gravity_force_s;
        
        Eigen::Vector3d r_hat_s(0, 0, 1);
        Eigen::Vector3d gravity_force_hat_0(0, 0, -1);
        Eigen::Vector3d gravity_force_hat_s_initial = s_R_0_initial_ * gravity_force_hat_0;
        Eigen::Vector3d gravity_torque_s_initial = max_gravity_torque * r_hat_s.cross(gravity_force_hat_s_initial);
        Eigen::Vector3d gravity_force_hat_s = s_R_0 * gravity_force_hat_0;
        Eigen::Vector3d gravity_torque_s = max_gravity_torque * r_hat_s.cross(gravity_force_hat_s);
        F_ext_S_s.tail(3) = sensor_data.tail(3) + gravity_torque_s_initial - gravity_torque_s;
        Eigen::Matrix<double, 6, 1> F_ext_EE_0;
        Eigen::Matrix3d O_R_s = s_R_0.transpose();
        F_ext_EE_0.head(3) = O_R_s * F_ext_S_s.head(3);
        Eigen::Vector3d r_O = O_R_s * r_s;
        Eigen::Vector3d F_O = F_ext_EE_0.head(3);
        F_ext_EE_0.tail(3) = O_R_s * F_ext_S_s.tail(3) + r_O.cross(F_O);
        if (!(bias_checked && !bias_error & !ecat_error)) {
            F_ext_EE_0.setZero();
            if (!bias_checked) {
                std::cout << "No bias checking" << std::endl;
            }
            if (bias_error) {
                std::cout << "Bias error" << std::endl;
            }
            if (ecat_error) {
                std::cout << "EtherCAT communication error" << std::endl;
            }
        }
        return F_ext_EE_0;
    }
    
    void locking::locking_unlocking(Eigen::Matrix<double,3,1> F_ext_O)
    {        
        if (locked_ == true){
            if (abs(F_ext_O[0])>force_threshold_ || abs(F_ext_O[1])>force_threshold_ || abs(F_ext_O[2])>force_threshold_){
                unlocking_counter_++;
                if (unlocking_counter_ > unlocking_counter_duration_){
                    locked_ = false;
                    unlocking_counter_ = 0;
                }
            }else{
                unlocking_counter_ = 0;
            }
        }else if (locked_ == false){
            if (abs(F_ext_O[0])<=force_threshold_ && abs(F_ext_O[1])<=force_threshold_ && abs(F_ext_O[2])<=force_threshold_){
                locking_counter_++;
                if (locking_counter_ > locking_counter_duration_){
                    locked_ = true;
                    locking_counter_ = 0;
                }
            }else{
                locking_counter_= 0;
            }
        }
    }

    
    bool initial_operations::initFrankaVelFT(hardware_interface::RobotHW* robot_hw, std::unique_ptr<franka_hw::FrankaStateHandle>* state_handle, std::unique_ptr<franka_hw::FrankaModelHandle>* model_handle,std::vector<hardware_interface::JointHandle>* joint_handles, panda_ecat_comm::ecatCommATIAxiaFTSensor& FT_sensor, EEPoleBaseFrameExtWrenchComputation& external_force_computation, int filter_level)
    {
        std::string arm_id ("panda");
        std::vector<std::string> joint_names = {"panda_joint1","panda_joint2","panda_joint3","panda_joint4","panda_joint5","panda_joint6","panda_joint7"};
        auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
        if (state_interface == nullptr) {
            ROS_ERROR_STREAM(
                "NullspaceController: Error getting state interface from hardware");
            return false;
        }
        try {
            *state_handle = std::make_unique<franka_hw::FrankaStateHandle>(
                state_interface->getHandle(arm_id + "_robot"));
        } catch (hardware_interface::HardwareInterfaceException& ex) {
            ROS_ERROR_STREAM(
                "NullspaceController: Exception getting state handle from interface: "
                << ex.what());
            return false;
        }
        
        auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
        if (model_interface == nullptr) {
            ROS_ERROR_STREAM(
                "NullspaceController: Error getting model interface from hardware");
            return false;
        }
        try {
            *model_handle = std::make_unique<franka_hw::FrankaModelHandle>(
                model_interface->getHandle(arm_id + "_model"));
        } catch (hardware_interface::HardwareInterfaceException& ex) {
            ROS_ERROR_STREAM(
                "NullspaceController: Exception getting model handle from interface: "
                << ex.what());
            return false;
        }

        auto* velocity_joint_interface = robot_hw->get<hardware_interface::VelocityJointInterface>();
        if (velocity_joint_interface == nullptr) {
            ROS_ERROR_STREAM(
                "NullspaceController: Error getting effort joint interface from hardware");
            return false;
        }
        for (size_t i = 0; i < 7; ++i) {
            try {
            (*joint_handles).push_back(velocity_joint_interface->getHandle(joint_names[i]));
            } catch (const hardware_interface::HardwareInterfaceException& ex) {
            ROS_ERROR_STREAM(
                "NullspaceController: Exception getting joint handles: " << ex.what());
            return false;
            }
        }
            char array[] = "enxa0cec88b0292";
            char *cstr = array;
            int slaves[1] = {2};
            FT_sensor.ecatinit(slaves,cstr,500);
            
        FT_sensor.send_control_code(0,0,filter_level,0,2);
        ros::Duration(1).sleep();
        FT_sensor.send_control_code(1,0,filter_level,0,2);
        ros::Duration(1).sleep();
        FT_sensor.send_control_code(0,0,filter_level,0,2);
        ros::Duration(0.1).sleep();
        
        franka::RobotState robot_state = (*state_handle)->getRobotState();
        Eigen::Map<Eigen::Matrix<double, 4, 4>> O_T_EE_initial(robot_state.O_T_EE.data());
        Eigen::Matrix<double, 3, 3> EE_R_O_initial = O_T_EE_initial.block(0, 0, 3, 3).transpose();
        external_force_computation.s_R_0_initial_ = external_force_computation.s_R_EE_ * EE_R_O_initial;

            return true;
    }
    
    void initial_operations::check_initial_bias(panda_ecat_comm::ecatCommATIAxiaFTSensor& FT_sensor) {
        Eigen::Matrix<double,6,1> FT_sensor_data = FT_sensor.get_FT_sensor_data();
        if (!bias_checked) {
            if (abs(FT_sensor_data[0])>0.2 || abs(FT_sensor_data[1])>0.2 || abs(FT_sensor_data[2])>0.2 || abs(FT_sensor_data[3])>0.2 || abs(FT_sensor_data[4])>0.2 || abs(FT_sensor_data[5])>0.2) {
                bias_error = true;
            }
            bias_checked = true;
        }
    }

}
