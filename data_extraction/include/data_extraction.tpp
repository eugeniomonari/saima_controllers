#include "data_extraction.h"
#include <fstream>

#define time_length 600000

namespace data_extraction
{
    template <typename data>
    DataExtraction<data>::DataExtraction()
    {
        data_buffer.reserve(time_length);
    }
    
    template <typename data>
    void DataExtraction<data>::update_data(std::unique_ptr<franka_hw::FrankaStateHandle> *state_handle, std::unique_ptr<franka_hw::FrankaModelHandle> *model_handle,const std::vector<Eigen::VectorXd>& custom_data)
    {
        data current_data;
        update_data_state(state_handle,&current_data);
        update_data_model(state_handle,model_handle,&current_data);
        if (custom_data.size() > 0)
        {
            update_data_custom(&current_data,custom_data);
        }
        data_buffer.push_back(current_data);
    }
    
    template <typename data>
    void DataExtraction<data>::update_data(const std::vector<Eigen::VectorXd>& custom_data)
    {
        data current_data;
        if (custom_data.size() > 0)
        {
            update_data_custom(&current_data,custom_data,true);
        }
        data_buffer.push_back(current_data);
    }
    
    template <typename data>
    void DataExtraction<data>::update_data_state(std::unique_ptr<franka_hw::FrankaStateHandle> *state_handle, data *current_data)
    {
        std::array<double,1> time_array;
        if (!initial_time_set_) {
            initial_time_ = (*state_handle)->getRobotState().time.toSec();
            initial_time_set_ = true;
        }
        time_array[0] = (*state_handle)->getRobotState().time.toSec()-initial_time_;
        std::get<0>(*current_data) = time_array;
        std::array<double,1> success_rate_array;
        success_rate_array[0] = (*state_handle)->getRobotState().control_command_success_rate;
        std::get<1>(*current_data) = success_rate_array;
        std::get<2>(*current_data) = (*state_handle)->getRobotState().O_T_EE;
        std::get<3>(*current_data) = (*state_handle)->getRobotState().tau_J;
        std::get<4>(*current_data) = (*state_handle)->getRobotState().tau_J_d;
        std::get<5>(*current_data) = (*state_handle)->getRobotState().q;
        std::get<6>(*current_data) = (*state_handle)->getRobotState().q_d;
        std::get<7>(*current_data) = (*state_handle)->getRobotState().dq;
        std::get<8>(*current_data) = (*state_handle)->getRobotState().dq_d;
        std::get<9>(*current_data) = (*state_handle)->getRobotState().ddq_d;
        std::array<std::string,1> errors_array;
        std::array<bool,1> errors_array_bool;
        if (!error_happened) {
            errors_array[0] = std::string((*state_handle)->getRobotState().current_errors);
            errors_array_bool[0] = bool((*state_handle)->getRobotState().current_errors);
            if (bool((*state_handle)->getRobotState().current_errors)) {
                error_happened = true;
            }
        }
        else {
            errors_array[0] = std::string((*state_handle)->getRobotState().last_motion_errors);
            errors_array_bool[0] = bool((*state_handle)->getRobotState().last_motion_errors);
        }        
        std::get<10>(*current_data) = errors_array;
        std::get<11>(*current_data) = errors_array_bool;
    }
    
    template<typename data>
    template<size_t I>
    void DataExtraction<data>::update_data_model(std::unique_ptr<franka_hw::FrankaStateHandle> *state_handle, std::unique_ptr<franka_hw::FrankaModelHandle> *model_handle, data *current_data)
    {
        if constexpr(I < std::tuple_size<state_datat>::value)
        {
            update_data_model<I+1>(state_handle,model_handle,current_data);
        }
        else if constexpr(I == std::tuple_size<state_datat>::value + 0)
        {
            std::get<I>(*current_data) = (*model_handle)->getZeroJacobian(franka::Frame::kEndEffector,(*state_handle)->getRobotState().q_d,(*state_handle)->getRobotState().F_T_EE,(*state_handle)->getRobotState().EE_T_K);
            update_data_model<I+1>(state_handle,model_handle,current_data);
        }
        else if constexpr(I == std::tuple_size<state_datat>::value + 1)
        {
            std::get<I>(*current_data) = (*model_handle)->getMass();
            update_data_model<I+1>(state_handle,model_handle,current_data);
        }
        else if constexpr(I == std::tuple_size<state_datat>::value + 2)
        {
            std::get<I>(*current_data) = (*model_handle)->getCoriolis();
            update_data_model<I+1>(state_handle,model_handle,current_data);
        }
        else if constexpr(I == std::tuple_size<state_datat>::value + 3)
        {
            std::get<I>(*current_data) = (*model_handle)->getGravity();
            update_data_model<I+1>(state_handle,model_handle,current_data);
        }
    }
    
    template<typename data>
    template<size_t I>
    void DataExtraction<data>::update_data_custom(data *current_data,const std::vector<Eigen::VectorXd>& custom_data,bool only_custom)
    {
        if(!only_custom)
        {
            if constexpr(I < std::tuple_size<state_datat>::value+std::tuple_size<model_datat>::value)
            {
                update_data_custom<I+1>(current_data,custom_data);
            }
            else if constexpr(I < std::tuple_size<data>::value)
            {
                for (int j = 0; j < custom_data[I-(std::tuple_size<state_datat>::value+std::tuple_size<model_datat>::value)].rows(); j++)
                {
                    std::get<I>(*current_data)[j] = custom_data[I-(std::tuple_size<state_datat>::value+std::tuple_size<model_datat>::value)][j];
                }
                update_data_custom<I+1>(current_data,custom_data);
            }
        }
        else
        {
            if constexpr(I < std::tuple_size<data>::value)
            {
                for (int j = 0; j < custom_data[I].rows(); j++)
                {
                    std::get<I>(*current_data)[j] = custom_data[I][j];
                }
                update_data_custom<I+1>(current_data,custom_data,only_custom);
            }
        }
    }
    
    template<typename data> void DataExtraction<data>::write_data_to_csv(std::vector<std::string> custom_headers,bool only_custom)
    {
        if (started)
        {
            time_t rawtime;
            struct tm *timeinfo;
            char buffer[80];
            std::time(&rawtime);
            timeinfo = localtime(&rawtime);
            strftime(buffer, sizeof(buffer), "%Y_%m_%d_%H_%M_%S_", timeinfo);
            std::string date(buffer);
            date_ = date;
            std::string filename = "/home/saima/recorded_data/recorded_data_" + date + ".csv";
            std::ofstream myfile;
            myfile.open(filename);
            std::vector<std::string> header_values;
            if (!only_custom)
            {
                header_values = state_header;
                header_values.insert(header_values.end(),model_header.begin(),model_header.end());
                if (custom_headers.size() > 0)
                {
                    header_values.insert(header_values.end(),custom_headers.begin(),custom_headers.end());
                }
            }
            else
            {
                header_values = custom_headers;
            }
            write_header(header_values);
            myfile << csv_header.str();
            for (size_t i = 0; i < data_buffer.size(); i++) // data_buffer.size()
            {
                write_csv_data_line(data_buffer[i]);
                
            }
            myfile << csv_data.str();
            myfile.close();
            ROS_INFO("Data successfully saved");
        }
    }
    
    template<typename data>
    template<size_t I>
    void DataExtraction<data>::write_header(std::vector<std::string> header_values)
    {
        if constexpr(I < std::tuple_size<data>::value)
        {
            typename std::tuple_element<I,data>::type Ith_data;
            for (size_t j = 0; j < Ith_data.size(); j++)
            {
                if (I == 0 && j == 0)
                {
                    csv_header << header_values[I] << "[" << j << "]";
                }
                else
                {
                    csv_header << "," << header_values[I] << "[" << j << "]";
                }
            }
            write_header<I+1>(header_values);
        }
        else csv_header << std::endl;
    }
    
    template<typename data>
    template<size_t I>
    void DataExtraction<data>::write_csv_data_line(data data_line)
    {
        if constexpr(I < std::tuple_size<data>::value)
        {
            typename std::tuple_element<I,data>::type Ith_data;
            Ith_data = std::get<I>(data_line);
            for (size_t j = 0; j < Ith_data.size(); j++)
            {
                if (I==0 && j==0)
                {
                    csv_data << std::setprecision(9) << Ith_data[j];
                }
                else
                {
                    csv_data << std::setprecision(9) << "," << Ith_data[j];
                }
            }
            write_csv_data_line<I+1>(data_line);
        }
        else 
        {
            csv_data << std::endl;
        }
    }

}
