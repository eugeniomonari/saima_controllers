#include "trial_controller.h"
#include <Eigen/Dense>
#include <pluginlib/class_list_macros.h>

#include </home/saima/eugenio/panda_robot/catkin_ws/optimizer2/optimizer2_bindings.hpp>

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
        if (!node_handle.getParam("optimization_params", optimization_params_) || optimization_params_.size() != 5) {
            ROS_ERROR_STREAM("Could not get optimization_params_ from parameter server");
            return false;
        }
        return true;
    }
    
    void TrialController::update(const ros::Time&, const ros::Duration& period)
    {
        data_extraction_.started = true;
        Eigen::Matrix<double,7,1> tau_d;
        tau_d.setZero();
        FT_sensor.send_control_code(0,0,0,0,2);
        Eigen::Matrix<double,6,1> FT_sensor_data;
        FT_sensor_data = FT_sensor.get_FT_sensor_data();
//         double p[OPTIMIZER2_NUM_PARAMETERS] = {0.10741, -0.093909, 0.16231, -0.69369, 0.15818, 0.54698, 0.40257, 
//             1.5165, -0.36189, -0.65596, 0.067858, -0.6358, -0.3132, 1,
//             0.023092, -0.016419, 0.030028, -0.099436, 0.31384, 0.088353, 0.070651,
//             0.0080922, -0.020169, 0.020028, -0.11194, 0.016384, 0.071225, 0.050651
//         };  // parameter
//         double p[OPTIMIZER2_NUM_PARAMETERS] = {-0.136708,-3.11535,-0.178112,-2.94078,-0.067503,-0.169963,-0.279653, 
//             -1.2501,0.00344211,1,-0.00222851,0.518094,0.00973201,-0.447004,
//             -0.444778,-0.0712359,0.354447,-0.0679523,0.192742,0.00927597,-0.160439,
//             -0.456301,-0.749859,0.344447,-0.0786667,0.177742,-0.010724,-0.180439
//         };  // parameter
//         double p[OPTIMIZER2_NUM_PARAMETERS] = {5.57331, 1.31427, 3.31041, -0.474413, 0.796842, 1.62737, 8.32948, 
//             -1.33303, -0.0259446, 1, -0.00100816, 0.632015, 0.00447423, -0.578616,
//             0.015, 0.0075, 0, 0.0125, 0, 0, 0.02,
//             -0.015, -0.0075, 0, -0.0125, 0, 0, -0.02
//         };  // parameter
        double p[OPTIMIZER2_NUM_PARAMETERS] = {-0.472945,    1.58622,  -0.102673,
   3.12353,  -0.401394,   0.244737,
  0.739632,   0.822039, -0.0794544,
   2.24648,  -0.205811,    2.72546,
  0.099346,   0.319294, -0.0797163,
  0.899771,  -0.216165,   -2.47902,
  0.300217,     2.2041, -0.0419733,
            
            0.154745, -2.27218, -0.653029,
            
            -1.33334, -0.0935949, 1, 0.00621655, 0.643086, -0.0277886, -0.604857,
            
            0.0075, 0.00375, 0.005, 0.00625, 0.0075, 0.01, 0.01,
            
            -0.0075, -0.00375, -0.005, -0.00625, -0.0075, -0.01, -0.01,
            
            optimization_params_[0], optimization_params_[1], optimization_params_[2], optimization_params_[3], optimization_params_[4]
        };  // parameter
        double u[OPTIMIZER2_NUM_DECISION_VARIABLES] = {10.0, 0.0, 0.0, 0.0, 0.0};  // initial guess
        
        optimizer2Cache *cache = optimizer2_new();
        optimizer2SolverStatus status = optimizer2_solve(cache, u, p, 0, 0);
        optimizer2_free(cache);
        if (!printed) {
        for (size_t i = 0; i < OPTIMIZER2_NUM_DECISION_VARIABLES; i++) {
            std::cout << i << " " << u[i] << std::endl;
        }
        std::cout << "time " << (double)status.solve_time_ns/1e6 << std::endl;
        std::cout << "inner iters " << status.num_inner_iterations << std::endl;
        std::cout << "outer iters " << status.num_outer_iterations << std::endl;
        std::cout << "exit_status " << (double)status.exit_status << std::endl;
//         printed = true;
        }
        
        for (size_t i = 0; i < 7; ++i)
        {
            joint_handles_[i].setCommand(tau_d(i));
        }
//         data_extraction_.update_data(&state_handle_,&model_handle_);
        std::vector<Eigen::VectorXd> custom_data(1);
        custom_data[0] = FT_sensor_data;
        data_extraction_.update_data(&state_handle_,&model_handle_,custom_data);
    }
    
    void TrialController::stopping(const ros::Time&)
    {
        std::vector<std::string> custom_header_values{"FT_sensor_data"};
        data_extraction_.write_data_to_csv(custom_header_values);
        data_extraction_.started = false;
    }
}

PLUGINLIB_EXPORT_CLASS(trial_controller::TrialController,controller_interface::ControllerBase)
