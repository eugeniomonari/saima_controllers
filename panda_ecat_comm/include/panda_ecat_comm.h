#pragma once

#include "ecat_comm.h"
#include "ecat_comm_devices.h"
#include <Eigen/Dense>

namespace panda_ecat_comm
{
    class ecatCommATIAxiaFTSensor : public ecat_comm::ecatComm<std::tuple<ecat_comm::in_ATIAxiaFTSensort>,std::tuple<ecat_comm::out_ATIAxiaFTSensort>>
    {
    public:
        Eigen::Matrix<double,6,1> get_F_ext_S_s();
        void send_control_code(int set_bias, int clear_bias, int filter, int calibration, int sample_rate);
    };
}
