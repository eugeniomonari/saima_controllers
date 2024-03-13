#include "panda_ecat_comm.h"

namespace panda_ecat_comm
{
    template<size_t I>
    Eigen::Matrix<double,6,1> get_F_ext_S_s_gen(void* ptr)
    {
        auto foo = static_cast<ecatCommATIAxiaFTSensor*>(ptr);
        Eigen::Matrix<double,6,1>F_ext_S_s;
        F_ext_S_s[0] = ((double)std::get<I>(foo->inputs).Fx)/1000000;
        F_ext_S_s[1] = ((double)std::get<I>(foo->inputs).Fy)/1000000;
        F_ext_S_s[2] = ((double)std::get<I>(foo->inputs).Fz)/1000000;
        F_ext_S_s[3] = ((double)std::get<I>(foo->inputs).Tx)/1000000;
        F_ext_S_s[4] = ((double)std::get<I>(foo->inputs).Ty)/1000000;
        F_ext_S_s[5] = ((double)std::get<I>(foo->inputs).Tz)/1000000;
        return F_ext_S_s;
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
    
    Eigen::Matrix<double,6,1> ecatCommATIAxiaFTSensor::get_F_ext_S_s()
    {
        return get_F_ext_S_s_gen<0>(this);
    }

}
