#include "data_extraction.h"
#include <fstream>

namespace data_extraction
{    
    template <typename data>
    void DataExtraction<data>::update_data(std::unique_ptr<franka_hw::FrankaStateHandle> *state_handle, std::unique_ptr<franka_hw::FrankaModelHandle> *model_handle,std::vector<Eigen::VectorXd> custom_data)
    {
        data current_data;
        update_data_state(state_handle,&current_data);
        update_data_model(model_handle,&current_data);
        if (custom_data.size() > 0)
        {
            update_data_custom(&current_data,custom_data);
        }
        data_buffer.push_back(current_data);
    }
    
    template <typename data>
    void DataExtraction<data>::update_data_state(std::unique_ptr<franka_hw::FrankaStateHandle> *state_handle, data *current_data)
    {
        std::get<0>(*current_data) = (*state_handle)->getRobotState().O_T_EE;
        std::get<1>(*current_data) = (*state_handle)->getRobotState().dq;
    }
    
    template<typename data>
    template<size_t I>
    void DataExtraction<data>::update_data_model(std::unique_ptr<franka_hw::FrankaModelHandle> *model_handle, data *current_data)
    {
        if constexpr(I < std::tuple_size<state_datat>::value)
        {
            update_data_model<I+1>(model_handle,current_data);
//             std::get<I>(current_data) = (*model_handle)->getZeroJacobian(franka::Frame::kEndEffector);
        }
        else if constexpr(I == std::tuple_size<state_datat>::value + 0)
        {
            std::get<I>(*current_data) = (*model_handle)->getZeroJacobian(franka::Frame::kEndEffector);
            update_data_model<I+1>(model_handle,current_data);
        }
    }
    
    template<typename data>
    template<size_t I>
    void DataExtraction<data>::update_data_custom(data *current_data,std::vector<Eigen::VectorXd> custom_data)
    {
        
        if constexpr(I < std::tuple_size<state_datat>::value+std::tuple_size<model_datat>::value)
        {
            update_data_custom<I+1>(current_data,custom_data);
        }
        else if constexpr(I < std::tuple_size<data>::value)
        {
//             std::cout << custom_data[I-(std::tuple_size<state_datat>::value+std::tuple_size<model_datat>::value)].rows() << std::endl;
            for (int j = 0; j < custom_data[I-(std::tuple_size<state_datat>::value+std::tuple_size<model_datat>::value)].rows(); j++)
            {
                std::get<I>(*current_data)[j] = custom_data[I-(std::tuple_size<state_datat>::value+std::tuple_size<model_datat>::value)][j];
            }
            update_data_custom<I+1>(current_data,custom_data);
        }
    }
    
    template<typename data> void DataExtraction<data>::write_data_to_csv(std::vector<std::string> custom_headers)
    {
        if (started)
        {
            time_t rawtime;
            struct tm *timeinfo;
            char buffer[80];
            std::time(&rawtime);
            timeinfo = localtime(&rawtime);
            strftime(buffer, sizeof(buffer), "%Y_%m_%d_%H_%M_%S_", timeinfo);
            std::string str(buffer);
            std::string filename = "/home/saima/recorded_data/recorded_data_" + str + ".csv";
            std::ofstream myfile;
            myfile.open(filename);
            std::vector<std::string> header_values = state_header;
            header_values.insert(header_values.end(),model_header.begin(),model_header.end());
            if (custom_headers.size() > 0)
            {
                header_values.insert(header_values.end(),custom_headers.begin(),custom_headers.end());
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
                    csv_data << Ith_data[j];
                }
                else
                {
                    csv_data << "," << Ith_data[j];
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
