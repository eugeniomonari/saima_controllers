#pragma once

#include <panda_ecat_comm.h>
#include <franka_hw/franka_state_interface.h>
#include <franka_hw/franka_model_interface.h>

namespace data_extraction
{
    typedef std::tuple<std::array<double,1>,std::array<double,1>,std::array<double,16>,std::array<double,7>,std::array<double,7>,std::array<double,7>,std::array<double,7>,std::array<double,7>,std::array<double,7>,std::array<double,7>,std::array<std::string,1>,std::array<bool,1>> state_datat;
    typedef std::tuple<std::array<double,42>,std::array<double,49>,std::array<double,7>,std::array<double,7>> model_datat;
    template<typename ... input_t>
    using tuple_cat_t=
    decltype(std::tuple_cat(
        std::declval<input_t>()...
    ));
    typedef tuple_cat_t<state_datat,model_datat> state_model_data_t;
    std::vector<std::string> state_header = {"time","success_rate","O_T_EE","tau_J","tau_J_d","q","q_d","dq","dq_d","ddq_d","errors","errors_bool"};
    std::vector<std::string> model_header = {"J","B","c","g"};
    template <typename data>
    class DataExtraction
    {
    public:
        std::vector<data> data_buffer;
        std::ostringstream csv_header;
        std::ostringstream csv_data;
        bool started = false;
        std::string date_;
        
        
        DataExtraction();
        void update_data(std::unique_ptr<franka_hw::FrankaStateHandle> *state_handle, std::unique_ptr<franka_hw::FrankaModelHandle> *model_handle, const std::vector<Eigen::VectorXd>& custom_data = std::vector<Eigen::VectorXd>());
        void update_data(const std::vector<Eigen::VectorXd>& custom_data = std::vector<Eigen::VectorXd>());
        void update_data_state(std::unique_ptr<franka_hw::FrankaStateHandle> *state_handle, data *current_data);
        template<size_t I = 0>
        void update_data_model(std::unique_ptr<franka_hw::FrankaStateHandle> *state_handle, std::unique_ptr<franka_hw::FrankaModelHandle> *model_handle, data *current_data);
        template<size_t I = 0>
        void update_data_custom(data *current_data,const std::vector<Eigen::VectorXd>& custom_data,bool only_custom = false);
        void write_data_to_csv(std::vector<std::string> custom_headers = std::vector<std::string>(),bool only_custom = false);
        template<size_t I = 0>
        void write_header(std::vector<std::string> header_values);
        template<size_t I = 0>
        void write_csv_data_line(data data_line);
        double initial_time_ = 0;
        bool initial_time_set_ = false;
        bool error_happened = false;
    };
}

#include <data_extraction.tpp>
