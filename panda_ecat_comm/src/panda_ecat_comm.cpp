#include "panda_ecat_comm.h"

namespace panda_ecat_comm
{
    int ecatCommATIAxiaFTSensor::set_bias = 0;
    int ecatCommATIAxiaFTSensor::filter = 0;
    int ecatCommATIAxiaFTSensor::m_slave_number = 0;
    ecat_comm::in_ATIAxiaFTSensort ecatCommATIAxiaFTSensor::read_data_ATIAxiaFTSensor;
    
    ecatCommATIAxiaFTSensor::ecatCommATIAxiaFTSensor()
    : ecat_comm::ecatComm<ecatCommATIAxiaFTSensor>()
    {
    }

    
    ecatCommATIAxiaFTSensor::ecatCommATIAxiaFTSensor(char* ifname, int ctime, int slave_number)
    : ecat_comm::ecatComm<ecatCommATIAxiaFTSensor>(ifname,ctime)
    {
        m_slave_number = slave_number;
    }


    void ecatCommATIAxiaFTSensor::ecat_read_write_PDO()
    {
        read_data_ATIAxiaFTSensor = ecat_comm::read_ATIAxiaFTSensor(m_slave_number);
        ecat_comm::write_ATIAxiaFTSensor(m_slave_number,set_bias,0,filter,0,1);
    }
}
