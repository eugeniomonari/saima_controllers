#pragma once

#include <panda_ecat_comm.h>
#include <franka_hw/franka_state_interface.h>
#include <franka_hw/franka_model_interface.h>

namespace data_extraction
{
    typedef std::tuple<std::array<double,16>,std::array<double,7>> state_datat; //O_T_EE,dq
    typedef std::tuple<std::array<double,42>> model_datat; //zeroJacobian
    template<typename ... input_t>
    using tuple_cat_t=
    decltype(std::tuple_cat(
        std::declval<input_t>()...
    ));
    typedef tuple_cat_t<state_datat,model_datat> state_model_data_t;
    std::vector<std::string> state_header = {"O_T_EE","dq"};
    std::vector<std::string> model_header = {"zeroJacobian"};
    template <typename data>
    class DataExtraction
    {
    public:
        std::vector<data> data_buffer;
        std::ostringstream csv_header;
        std::ostringstream csv_data;
        bool started = false;
        
        void update_data(std::unique_ptr<franka_hw::FrankaStateHandle> *state_handle, std::unique_ptr<franka_hw::FrankaModelHandle> *model_handle, std::vector<Eigen::VectorXd> custom_data = std::vector<Eigen::VectorXd>());
        void update_data_state(std::unique_ptr<franka_hw::FrankaStateHandle> *state_handle, data *current_data);
        template<size_t I = 0>
        void update_data_model(std::unique_ptr<franka_hw::FrankaModelHandle> *model_handle, data *current_data);
        template<size_t I = 0>
        void update_data_custom(data *current_data,std::vector<Eigen::VectorXd> custom_data);
        void write_data_to_csv(std::vector<std::string> custom_headers = std::vector<std::string>());
        template<size_t I = 0>
        void write_header(std::vector<std::string> header_values);
        template<size_t I = 0>
        void write_csv_data_line(data data_line);
    };
}

#include <data_extraction.tpp>
