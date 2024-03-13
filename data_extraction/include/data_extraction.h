// #pragma once
// 
// #include <controller_interface/multi_interface_controller.h>
// #include <hardware_interface/robot_hw.h>
// #include <ros/node_handle.h>
// #include <ros/time.h>
// #include <franka_hw/franka_state_interface.h>
// #include <franka_hw/franka_model_interface.h>
// #include <panda_ecat_comm.h>
// 
// #include <boost/circular_buffer.hpp>
// 
// 
// namespace data_extraction
// {
//     
//     struct RobotModel {
//         std::array<double, 16> pose;
//         std::array<double, 42> zeroJacobian;
//         std::array<double, 7> gravity;
//         std::array<double, 7> coriolis;
//         std::array<double, 49> mass;
//     };
//     
//     struct RobotCommand {
//         franka::JointPositions joint_positions{0, 0, 0, 0, 0, 0, 0};
//         franka::JointVelocities joint_velocities{0, 0, 0, 0, 0, 0, 0};
//         franka::CartesianPose cartesian_pose{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
//         franka::CartesianVelocities cartesian_velocities{0, 0, 0, 0, 0, 0};
//         franka::Torques torques{0, 0, 0, 0, 0, 0, 0};
//         std::array<double, 7> dq_d_code{0, 0, 0, 0, 0, 0, 0};
//         std::array<bool, 1> locked{true};
//     };
//     
//     struct RecordData
//     {
//         double time;
//         franka::RobotState state;
//         RobotModel model;
//         RobotCommand command;
//         Eigen::Matrix<double, 6, 1> F_ext_S_s; 
//     };
//     
//     class DataExtraction : public controller_interface::MultiInterfaceController<franka_hw::FrankaStateInterface,franka_hw::FrankaModelInterface>
//     {
//     public: 
//         bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
//         void update(const ros::Time&, const ros::Duration& period) override;
//     private:
//         std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
//         std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
//         panda_ecat_comm::ecatCommATIAxiaFTSensor FT_sensor;
//     };
// }
