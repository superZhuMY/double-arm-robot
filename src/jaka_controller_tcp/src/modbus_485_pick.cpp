#include <stdexcept>
#include <iostream>
#include "modbus_485_pick.h"
namespace modbus {
    // modbus_rtu构造函数
    modbus_rtu::modbus_rtu( const std::string& device,   //串口设备，类似于："/dev/ttyUSB0"
                    int baud,                    //波特率：9600 115200等等
                    char parity,                 //奇偶校验位：通常设置为'N'
                    int data_bit,                //数据位：通常设置为8
                    int stop_bit,
                    int addr)                 //停止位：通常设置为1
        : ctx_(modbus_new_rtu(device.c_str(), baud,parity,data_bit,stop_bit))
    {
        if (MODBUS_ERR == modbus_set_slave(ctx_, addr) 
        || MODBUS_ERR == modbus_connect(ctx_))
        {
            modbus_free(ctx_);
            throw std::runtime_error("modbus_rtu init or connect failed");
        };
        this->write_single_register(0x0000, 0x0001);
        std::cout<<"modbus_rtu构造函数调用"<<std::endl;
    }

    // modbus_rtu析构函数
    modbus_rtu::~modbus_rtu()
    {
        modbus_close(ctx_);
        modbus_free(ctx_);
        std::cout<<"modbus_rtu析构函数调用"<<std::endl;
    }

    //0x01：读线圈状态: 00 01 00 00 00 06 01        01 67 89 00 05 
    int modbus_rtu::read_coil_status(int addr, int nb, uint8_t *dest)
    {
        return modbus_read_bits(ctx_, addr, nb, dest);       // return the number of read bits if successful. Otherwise it shall return -1 and set errno.
    }
    //0x02：读离散量输入: 00 01 00 00 00 06 01      02 67 89 00 05 
    int modbus_rtu::read_input_status(int addr, int nb, uint8_t *dest)
    {
        return modbus_read_input_bits(ctx_, addr, nb, dest);  // return the number of read input status if successful. Otherwise it shall return -1 and set errno.
    }
    //0x03：读保持寄存器: 00 01 00 00 00 06 01      03 67 89 00 05 
    int modbus_rtu::read_holding_register(int addr, int nb, uint16_t *dest)
    {
        return modbus_read_registers(ctx_, addr, nb, dest);   //成功时返回读取输入位的数目即nb，失败时返回-1.
    }
    //0x04：读输入寄存器: 00 01 00 00 00 06 01      04 67 89 00 05
    int modbus_rtu::read_input_register(int addr, int nb, uint16_t *dest)
    {
        return modbus_read_input_registers(ctx_, addr, nb, dest);  //return the number of read input registers if successful. Otherwise it shall return -1 and set errno.
    }
    //0x05：写单个线圈: 00 01 00 00 00 06 01        05 67 89 FF 00 
    int modbus_rtu::write_single_coil(int coil_addr, int status)
    {
        return modbus_write_bit(ctx_, coil_addr, status);  //成功时，返回1；失败时，返回-1。
    }
    //0x06：写单个保持寄存器: 00 01 00 00 00 06 01    06 67 89 12 34
    int modbus_rtu::write_single_register(int reg_addr, const uint16_t value)
    {
        return modbus_write_register(ctx_, reg_addr, value); //成功时返回1，失败时返回-1.
    }
    //0x0F（15）：写多个线圈: 00 01 00 00 00 08 01     0F 67 89 00 05 01 1F
    int modbus_rtu::write_multiple_coil(int addr, int nb, const uint8_t *data)
    {
        return modbus_write_bits(ctx_, addr, nb, data); //return the number of written bits if successful. Otherwise it shall return -1 and set errno.
    }
    //0x10（16）：写多个保持寄存器: 00 01 00 00 00 11 01   10 67 89 00 05 0A 00 00 00 00 00 00 00 00 00 05 
    int modbus_rtu::write_multiple_registers(int addr, int nb, const uint16_t *data)
    {
        int a=modbus_write_registers(ctx_, addr, nb, data);
        a=modbus_write_registers(ctx_, addr, nb, data);
        a=modbus_write_registers(ctx_, addr, nb, data);
        return a;  //成功时返回写入寄存器个数即nb，失败时返回-1.
    }
    void modbus_rtu::decimalToHex(unsigned int decimal,uint16_t *data){
        unsigned int high = (decimal >> 16) & 0xFFFF;
        unsigned int low = decimal & 0xFFFF; 
        data[0]=low;
        data[1]=high;
        return;
    }
    // void modbus_rtu::decimalToHex(unsigned int decimal,uint16_t *data){
    //     unsigned int high = (decimal >> 16) & 0xFFFF;
    //     unsigned int low = decimal & 0xFFFF; 
    //     data[0]=low;
    //     data[1]=high;
    //     return;
    // }



    unsigned int modbus_rtu::hexToDecimal1(uint16_t *data) {
        unsigned int low = data[0];
        unsigned int high = data[1];
        return (high << 16) | low;
    }


    int modbus_rtu::hexToDecimal(uint16_t *data) 
    {
        uint16_t low = data[0];
        uint16_t high = data[1];
        uint16_t data1[2];
        if (high>61440){
            data1[0]=0xFFFF-data[0]+0x0001;
            data1[1]=0xFFFF-data[1];
            int aa=this->hexToDecimal1(data1);
            int bb=-aa;
            return bb;
        }
        else{
            return this->hexToDecimal1(data);
        }
    }




    void modbus_rtu::decimalToHexx(unsigned int decimal, uint16_t *data){
        uint16_t hex_value = decimal & 0xFFFF; // 只保留低 16 位作为十六进制值
        *data = hex_value; // 将十六进制值存储到数组中
        return;
    }
    
    
    void modbus_rtu::Set_servo_frequency(unsigned int decimal,int addr){
        uint16_t data[1];
        this->decimalToHex(decimal, data);
        uint16_t data1[1];
        data1[0]=data[1];
        data1[1]=data[0];

        this->write_multiple_registers(addr, 2,data1);
        this->write_multiple_registers(addr, 2,data1);
        return;
    }




}  
