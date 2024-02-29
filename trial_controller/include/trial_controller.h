#pragma once

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <franka_hw/franka_state_interface.h>
#include <panda_ecat_comm.h>

namespace trial_controller
{
    class TrialController : public controller_interface::MultiInterfaceController<hardware_interface::EffortJointInterface, franka_hw::FrankaStateInterface>
    {
    public: 
        bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
        void update(const ros::Time&, const ros::Duration& period) override;
    private:
        std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
        std::vector<hardware_interface::JointHandle> joint_handles_;
        panda_ecat_comm::ecatCommATIAxiaFTSensor FT_sensor;
    };
}
