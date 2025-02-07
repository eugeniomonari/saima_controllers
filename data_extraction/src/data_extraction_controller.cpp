#include "data_extraction_controller.h"
#include <pluginlib/class_list_macros.h>
#include <franka/rate_limiting.h>
#include <chrono>
#include <ctime>

namespace data_extraction
{
    bool DataExtractionController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle)
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
            state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
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
            model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
                model_interface->getHandle(arm_id + "_model"));
        } catch (hardware_interface::HardwareInterfaceException& ex) {
            ROS_ERROR_STREAM(
                "NullspaceController: Exception getting model handle from interface: "
                << ex.what());
            return false;
        }

        std::string eth_interface_name = "";
        if (!node_handle.getParam("eth_interface_name", eth_interface_name)) {
            ROS_ERROR_STREAM("HandGuidanceController: could not get FT_filter_level from parameter server");
            return false;
        }
        char array[1024];
        strcpy(array, eth_interface_name.c_str());
        char *cstr = array;
        int slaves[1] = {2};
        FT_sensor.ecatinit(slaves,cstr,500);
        
        double filter_level = 0;
        FT_sensor.send_control_code(0,0,filter_level,0,2);
        ros::Duration(1).sleep();
        FT_sensor.send_control_code(1,0,filter_level,0,2);
        ros::Duration(1).sleep();
        FT_sensor.send_control_code(0,0,filter_level,0,2);
        ros::Duration(0.1).sleep();
        
        time_t rawtime;
        struct tm *timeinfo;
        std::time(&rawtime);
        timeinfo = localtime(&rawtime);
        strftime(data_extraction_.sequential_date_, sizeof(data_extraction_.sequential_date_), "%Y_%m_%d_%H_%M_%S_", timeinfo);
        
        return true;
    }
    
    void DataExtractionController::update(const ros::Time&, const ros::Duration& period)
    {
        initOperations.check_initial_bias(FT_sensor);
        data_extraction_.started = true;
        Eigen::Matrix<double,6,1> F_ext_S_s;
        F_ext_S_s = FT_sensor.get_FT_sensor_data();
        std::vector<Eigen::VectorXd> custom_data(1);
        custom_data[0] = F_ext_S_s;
        data_extraction_.update_data(&state_handle_,&model_handle_,custom_data);
    }
    
    void DataExtractionController::stopping(const ros::Time&)
    {
        std::vector<std::string> custom_header_values{"F_ext_S_s"};
        data_extraction_.write_data_to_csv_sequential(custom_header_values);
        data_extraction_.started = false;
        data_extraction_.data_buffer.clear();
    }
    
}

PLUGINLIB_EXPORT_CLASS(data_extraction::DataExtractionController,controller_interface::ControllerBase)

