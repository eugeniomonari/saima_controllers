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

namespace trial_controller
{
    class TrialController : public controller_interface::MultiInterfaceController<hardware_interface::EffortJointInterface, franka_hw::FrankaStateInterface,franka_hw::FrankaModelInterface>
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
        typedef std::tuple<std::array<double,6>> custom_data_t;
        typedef data_extraction::tuple_cat_t<data_extraction::state_model_data_t,custom_data_t> state_model_custom_data_t;
        data_extraction::DataExtraction<state_model_custom_data_t> data_extraction_;
//         data_extraction::DataExtraction<data_extraction::state_model_data_t> data_extraction_;
        std::vector<double> optimization_params_;
        bool printed = false;
    };
}
