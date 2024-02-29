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
                "CartesianImpedanceExampleController: Error getting state interface from hardware");
            return false;
        }
        try {
            state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
                state_interface->getHandle(arm_id + "_robot"));
        } catch (hardware_interface::HardwareInterfaceException& ex) {
            ROS_ERROR_STREAM(
                "CartesianImpedanceExampleController: Exception getting state handle from interface: "
                << ex.what());
            return false;
        }

        auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
        if (effort_joint_interface == nullptr) {
            ROS_ERROR_STREAM(
                "CartesianImpedanceExampleController: Error getting effort joint interface from hardware");
            return false;
        }
        for (size_t i = 0; i < 7; ++i) {
            try {
            joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
            } catch (const hardware_interface::HardwareInterfaceException& ex) {
            ROS_ERROR_STREAM(
                "CartesianImpedanceExampleController: Exception getting joint handles: " << ex.what());
            return false;
            }
        }
        char array[] = "enx00e04c6875c3";
        char *cstr = array;
        FT_sensor.~ecatCommATIAxiaFTSensor();
        new(&FT_sensor) panda_ecat_comm::ecatCommATIAxiaFTSensor(cstr,5000,5);
        FT_sensor.set_bias = 1;
        FT_sensor.filter = 0;
        return true;
    }
    
    void TrialController::update(const ros::Time&, const ros::Duration& period)
    {
        Eigen::Matrix<double,7,1> tau_d;
        tau_d.setZero();
        for (size_t i = 0; i < 7; ++i)
        {
            FT_sensor.set_bias = 0;
            std::cout << (((double)FT_sensor.read_data_ATIAxiaFTSensor.Fz)/1000000)/9.80665 << std::endl;
            joint_handles_[i].setCommand(tau_d(i));
        }
    }
}

PLUGINLIB_EXPORT_CLASS(trial_controller::TrialController,controller_interface::ControllerBase)
