#pragma once

#include "ecat_comm.h"
#include "ecat_comm_devices.h"

namespace panda_ecat_comm
{
    class ecatCommATIAxiaFTSensor : public ecat_comm::ecatComm<ecatCommATIAxiaFTSensor>
    {
    public:
        static int m_slave_number;
        static int set_bias;
        static int filter;
        static ecat_comm::in_ATIAxiaFTSensort read_data_ATIAxiaFTSensor;
        
        ecatCommATIAxiaFTSensor();
        ecatCommATIAxiaFTSensor(char *ifname, int ctime, int slave_number);
        static void ecat_read_write_PDO();
    };
}
