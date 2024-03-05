#include "panda_ecat_comm.h"

namespace panda_ecat_comm
{
    Eigen::Matrix<double,6,1> ecatCommATIAxiaFTSensor::get_F_ext_S_s()
    {
        Eigen::Matrix<double,6,1>F_ext_S_s;
        F_ext_S_s[0] = ((double)std::get<0>(inputs).Fx)/1000000;
        F_ext_S_s[1] = ((double)std::get<0>(inputs).Fy)/1000000;
        F_ext_S_s[2] = ((double)std::get<0>(inputs).Fz)/1000000;
        F_ext_S_s[3] = ((double)std::get<0>(inputs).Tx)/1000000;
        F_ext_S_s[4] = ((double)std::get<0>(inputs).Ty)/1000000;
        F_ext_S_s[5] = ((double)std::get<0>(inputs).Tz)/1000000;
        return F_ext_S_s;
    }
    
    void ecatCommATIAxiaFTSensor::send_control_code(int set_bias, int clear_bias, int filter, int calibration, int sample_rate)
    {
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
        
        std::get<0>(outputs).Control1 = control_code;
        std::get<0>(outputs).Control2 = 0;
    }
}
