#include "trial_controller.h"
#include <Eigen/Dense>
#include <pluginlib/class_list_macros.h>

namespace trial_controller
{
    bool TrialController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle)
    {
        std::string arm_id;
        if (!node_handle.getParam("arm_id", arm_id)) {
            ROS_ERROR_STREAM("TrialController: Could not read parameter arm_id");
            return false;
        }
        std::vector<std::string> joint_names;
        if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
            ROS_ERROR(
                "TrialController: Invalid or no joint_names parameters provided, "
                "aborting controller init!");
            return false;
        }
        auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
        if (state_interface == nullptr) {
            ROS_ERROR_STREAM(
                "TrialController: Error getting state interface from hardware");
            return false;
        }
        try {
            state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
                state_interface->getHandle(arm_id + "_robot"));
        } catch (hardware_interface::HardwareInterfaceException& ex) {
            ROS_ERROR_STREAM(
                "TrialController: Exception getting state handle from interface: "
                << ex.what());
            return false;
        }
        
        auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
        if (model_interface == nullptr) {
            ROS_ERROR_STREAM(
                "TrialController: Error getting model interface from hardware");
            return false;
        }
        try {
            model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
                model_interface->getHandle(arm_id + "_model"));
        } catch (hardware_interface::HardwareInterfaceException& ex) {
            ROS_ERROR_STREAM(
                "TrialController: Exception getting model handle from interface: "
                << ex.what());
            return false;
        }

        auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
        if (effort_joint_interface == nullptr) {
            ROS_ERROR_STREAM(
                "TrialController: Error getting effort joint interface from hardware");
            return false;
        }
        for (size_t i = 0; i < 7; ++i) {
            try {
            joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
            } catch (const hardware_interface::HardwareInterfaceException& ex) {
            ROS_ERROR_STREAM(
                "TrialController: Exception getting joint handles: " << ex.what());
            return false;
            }
        }
        char array[] = "enx00e04c6875c3";
        char *cstr = array;
        int slaves[1] = {2};
        FT_sensor.ecatinit(slaves,cstr,500);
        FT_sensor.send_control_code(1,0,0,0,2);
        ros::Duration(0.01).sleep();
        return true;
    }
    
    void TrialController::update(const ros::Time&, const ros::Duration& period)
    {
        data_extraction_.started = true;
        Eigen::Matrix<double,7,1> tau_d;
        tau_d.setZero();
        FT_sensor.send_control_code(0,0,0,0,2);
        Eigen::Matrix<double,6,1> F_ext_S_s;
        F_ext_S_s = FT_sensor.get_F_ext_S_s();
        for (size_t i = 0; i < 7; ++i)
        {
            joint_handles_[i].setCommand(tau_d(i));
        }
//         data_extraction_.update_data(&state_handle_,&model_handle_);
        std::vector<Eigen::VectorXd> custom_data(1);
        custom_data[0] = F_ext_S_s;
        data_extraction_.update_data(&state_handle_,&model_handle_,custom_data);
    }
    
    void TrialController::stopping(const ros::Time&)
    {
        std::vector<std::string> custom_header_values{"F_ext_S_s"};
        data_extraction_.write_data_to_csv(custom_header_values);
        data_extraction_.started = false;
    }
}

PLUGINLIB_EXPORT_CLASS(trial_controller::TrialController,controller_interface::ControllerBase)
