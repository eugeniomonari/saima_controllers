#pragma once

#include "ecat_comm.h"
#include "ecat_comm_devices.h"
#include <Eigen/Dense>

namespace panda_ecat_comm
{
    template<size_t I>
    Eigen::Matrix<double,6,1> get_FT_sensor_data_gen(void* ptr);
    template<size_t I>
    void send_control_code_gen(int set_bias, int clear_bias, int filter, int calibration, int sample_rate, void *ptr);
    
    class ecatCommATIAxiaFTSensor : public ecat_comm::ecatComm<std::tuple<ecat_comm::in_ATIAxiaFTSensort>,std::tuple<ecat_comm::out_ATIAxiaFTSensort>>
    {
    public :
        void send_control_code(int set_bias, int clear_bias, int filter, int calibration, int sample_rate);
        Eigen::Matrix<double,6,1> get_FT_sensor_data();
    };
}
