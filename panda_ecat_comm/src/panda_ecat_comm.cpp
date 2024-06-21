#include "panda_ecat_comm.h"

namespace panda_ecat_comm
{
    template<size_t I>
    Eigen::Matrix<double,6,1> get_FT_sensor_data_gen(void* ptr)
    {
        auto foo = static_cast<ecatCommATIAxiaFTSensor*>(ptr);
        Eigen::Matrix<double,6,1>FT_sensor_data;
        if (!foo->error) {
            FT_sensor_data[0] = ((double)std::get<I>(foo->inputs).Fx)/1000000;
            FT_sensor_data[1] = ((double)std::get<I>(foo->inputs).Fy)/1000000;
            FT_sensor_data[2] = ((double)std::get<I>(foo->inputs).Fz)/1000000;
            FT_sensor_data[3] = ((double)std::get<I>(foo->inputs).Tx)/1000000;
            FT_sensor_data[4] = ((double)std::get<I>(foo->inputs).Ty)/1000000;
            FT_sensor_data[5] = ((double)std::get<I>(foo->inputs).Tz)/1000000;
        }
        else {
            FT_sensor_data[0] = 0;
            FT_sensor_data[1] = 0;
            FT_sensor_data[2] = 0;
            FT_sensor_data[3] = 0;
            FT_sensor_data[4] = 0;
            FT_sensor_data[5] = 0;
        }
        return FT_sensor_data;
    }
    
    template<size_t I>
    void send_control_code_gen(int set_bias, int clear_bias, int filter, int calibration, int sample_rate, void* ptr)
    {
        auto foo = static_cast<ecatCommATIAxiaFTSensor*>(ptr);
        uint32_t control_code = 0;
        if (set_bias <= 1)
        {
            control_code = control_code + set_bias*std::pow(2,0);
        }
        else
        {
            std::cout << "out_ATIAxiaFTSensor: invalid control code for set_bias." << std::endl;
        }
        if (clear_bias <= 1)
        {
            control_code = control_code + clear_bias*std::pow(2,2);
        }
        else
        {
            std::cout << "out_ATIAxiaFTSensor: invalid control code for clear_bias." << std::endl;
        }
        if (filter <= 8)
        {
            control_code = control_code + filter*std::pow(2,4);
        }
        else
        {
            std::cout << "out_ATIAxiaFTSensor: invalid control code for filter." << std::endl;
        }
        if (calibration <= 1)
        {
            control_code = control_code + calibration*std::pow(2,8);
        }
        else
        {
            std::cout << "out_ATIAxiaFTSensor: invalid control code for calibration." << std::endl;
        }
        if (sample_rate <= 3)
        {
            control_code = control_code + sample_rate*std::pow(2,12);
        }
        else
        {
            std::cout << "out_ATIAxiaFTSensor: invalid control code for sample_rate." << std::endl;
        }
        
        std::get<I>(foo->outputs).Control1 = control_code;
        std::get<I>(foo->outputs).Control2 = 0;
    }
    
    void ecatCommATIAxiaFTSensor::send_control_code(int set_bias, int clear_bias, int filter, int calibration, int sample_rate)
    {
        send_control_code_gen<0>(set_bias,clear_bias,filter,calibration,sample_rate,this);
    }
    
    Eigen::Matrix<double,6,1> ecatCommATIAxiaFTSensor::get_FT_sensor_data()
    {
        return get_FT_sensor_data_gen<0>(this);
    }

}
