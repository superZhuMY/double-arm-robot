/**
 * @brief  modbus rtu base class
 */
#ifndef __MODBUS_RTU_HPP__
#define __MODBUS_RTU_HPP__

#include <chrono>
#include <string>

#include "modbus/modbus.h"

namespace modbus {
    /**
     * modbus_rtu base class
     */
    class modbus_rtu
    {
    public:
        /**
         * @brief       Constructor
         * @throw       @ref std::runtime_error() on gripper modbus communication failure
         *
         * @param[in]   ip      The modbus ip of gripper
         * @param[in]   port    The modbus rtu port of gripper
         */
        modbus_rtu( const std::string& device,   //串口设备，类似于："/dev/ttyUSB0"
                    int baud,                    //波特率：9600 115200等等
                    char parity,                 //奇偶校验位：通常设置为'N'
                    int data_bit,                //数据位：通常设置为8
                    int stop_bit,
                    int addr);
        ~modbus_rtu();

        //! libmodbus's error return value
        static constexpr int        MODBUS_ERR = -1;

        //0x01：读线圈状态: 00 01 00 00 00 06 01        01 67 89 00 05 
        int read_coil_status(int addr, int nb, uint8_t *dest);
        //0x02：读离散量输入: 00 01 00 00 00 06 01      02 67 89 00 05 
        int read_input_status(int addr, int nb, uint8_t *dest);
        //0x03：读保持寄存器: 00 01 00 00 00 06 01      03 67 89 00 05 
        int read_holding_register(int addr, int nb, uint16_t *dest);
        //0x04：读输入寄存器: 00 01 00 00 00 06 01      04 67 89 00 05
        int read_input_register(int addr, int nb, uint16_t *dest);
        //0x05：写单个线圈: 00 01 00 00 00 06 01        05 67 89 FF 00 
        int write_single_coil(int coil_addr, int status);
        //0x06：写单个保持寄存器: 00 01 00 00 00 06 01    06 67 89 12 34
        int write_single_register(int reg_addr, const uint16_t value);
        //0x0F（15）：写多个线圈: 00 01 00 00 00 08 01     0F 67 89 00 05 01 1F
        int write_multiple_coil(int addr, int nb, const uint8_t *data);
        //0x10（16）：写多个保持寄存器: 00 01 00 00 00 11 01   10 67 89 00 05 0A 00 00 00 00 00 00 00 00 00 05 
        int write_multiple_registers(int addr, int nb, const uint16_t *data);
        void decimalToHex(unsigned int decimal,  uint16_t *data);
        unsigned int hexToDecimal1(uint16_t *data);
        int hexToDecimal(uint16_t *data);
        void decimalToHexx(unsigned int decimal, uint16_t *data);
        void Set_servo_frequency(unsigned int decimal,int addr);
        modbus_t* ctx_;
        
    };
}  // namespace modbus
#endif
