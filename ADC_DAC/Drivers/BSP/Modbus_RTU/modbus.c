#include "modbus.h"
#include <string.h>

// 设置电流变量
extern uint8_t bytes[4]; // 浮点数电压转换成4字节表示

static UART_HandleTypeDef *ModbusUart;

// 串口句柄
void Modbus_Init(UART_HandleTypeDef *huart)
{
    ModbusUart = huart;
}

// 接收帧超时检测
ModbusStatus Modbus_Receive(uint8_t *rxBuffer, uint16_t len)
{
    if (HAL_UART_Receive(ModbusUart, rxBuffer, len, 1000) == HAL_OK)
    {
        return MODBUS_OK;
    }
    return MODBUS_TIMEOUT;
}

// Modbus 帧处理函数
ModbusStatus Modbus_Process(uint8_t *rxBuffer, uint16_t len, uint8_t *txBuffer, uint16_t *txLen)
{

    txBuffer[0] = 0x01;
    txBuffer[1] = 0x10;
    txBuffer[2] = 0x00;
    txBuffer[3] = 0x09;
    txBuffer[4] = 0x00;
    txBuffer[5] = 0x05;
    txBuffer[6] = 0x0A;
    txBuffer[7] = 0x00;
    txBuffer[8] = 0x03;
    txBuffer[9] = 0x00;
    txBuffer[10] = 0x00;
    txBuffer[11] = bytes[3];
    txBuffer[12] = bytes[2];
    txBuffer[13] = bytes[1];
    txBuffer[14] = bytes[0];
    txBuffer[15] = 0x41;
    txBuffer[16] = 0x70;
    txBuffer[17] = 0x00;
    txBuffer[18] = 0x00;

    // 增加校验位
    uint16_t response_crc = CRC16_Calculate(txBuffer, 19);
    txBuffer[19] = response_crc & 0xFF;
    txBuffer[20] = response_crc >> 8;

    // 设置数据长度
    *txLen = 19 + 2; // 数据长度 + CRC

    return MODBUS_OK;
}

// Modbus 帧处理函数
ModbusStatus Modbus_Process_Set_72V(uint8_t *rxBuffer, uint16_t len, uint8_t *txBuffer, uint16_t *txLen)
{

    txBuffer[0] = 0x01;
    txBuffer[1] = 0x10;
    txBuffer[2] = 0x00;
    txBuffer[3] = 0x09;
    txBuffer[4] = 0x00;
    txBuffer[5] = 0x05;
    txBuffer[6] = 0x0A;
    txBuffer[7] = 0x00;
    txBuffer[8] = 0x03;
    txBuffer[9] = 0x00;
    txBuffer[10] = 0x00;
    txBuffer[11] = 0x42;
    txBuffer[12] = 0x90;
    txBuffer[13] = 0x00;
    txBuffer[14] = 0x00;
    txBuffer[15] = 0x41;
    txBuffer[16] = 0x70;
    txBuffer[17] = 0x00;
    txBuffer[18] = 0x00;

    // 增加校验位
    uint16_t response_crc = CRC16_Calculate(txBuffer, 19);
    txBuffer[19] = response_crc & 0xFF;
    txBuffer[20] = response_crc >> 8;

    // 设置数据长度
    *txLen = 19 + 2; // 数据长度 + CRC

    return MODBUS_OK;
}

// Modbus 帧处理函数
ModbusStatus Modbus_Process_Set_48V(uint8_t *rxBuffer, uint16_t len, uint8_t *txBuffer, uint16_t *txLen)
{

    txBuffer[0] = 0x01;
    txBuffer[1] = 0x10;
    txBuffer[2] = 0x00;
    txBuffer[3] = 0x09;
    txBuffer[4] = 0x00;
    txBuffer[5] = 0x05;
    txBuffer[6] = 0x0A;
    txBuffer[7] = 0x00;
    txBuffer[8] = 0x03;
    txBuffer[9] = 0x00;
    txBuffer[10] = 0x00;
    txBuffer[11] = 0x42;
    txBuffer[12] = 0x40;
    txBuffer[13] = 0x00;
    txBuffer[14] = 0x00;
    txBuffer[15] = 0x41;
    txBuffer[16] = 0x70;
    txBuffer[17] = 0x00;
    txBuffer[18] = 0x00;

    // 增加校验位
    uint16_t response_crc = CRC16_Calculate(txBuffer, 19);
    txBuffer[19] = response_crc & 0xFF;
    txBuffer[20] = response_crc >> 8;

    // 设置数据长度
    *txLen = 19 + 2; // 数据长度 + CRC

    return MODBUS_OK;
}

// Modbus 帧处理函数
ModbusStatus Modbus_Process_Set_36V(uint8_t *rxBuffer, uint16_t len, uint8_t *txBuffer, uint16_t *txLen)
{

    txBuffer[0] = 0x01;
    txBuffer[1] = 0x10;
    txBuffer[2] = 0x00;
    txBuffer[3] = 0x09;
    txBuffer[4] = 0x00;
    txBuffer[5] = 0x05;
    txBuffer[6] = 0x0A;
    txBuffer[7] = 0x00;
    txBuffer[8] = 0x03;
    txBuffer[9] = 0x00;
    txBuffer[10] = 0x00;
    txBuffer[11] = 0x42;
    txBuffer[12] = 0x10;
    txBuffer[13] = 0x00;
    txBuffer[14] = 0x00;
    txBuffer[15] = 0x41;
    txBuffer[16] = 0x70;
    txBuffer[17] = 0x00;
    txBuffer[18] = 0x00;

    // 增加校验位
    uint16_t response_crc = CRC16_Calculate(txBuffer, 19);
    txBuffer[19] = response_crc & 0xFF;
    txBuffer[20] = response_crc >> 8;

    // 设置数据长度
    *txLen = 19 + 2; // 数据长度 + CRC

    return MODBUS_OK;
}

// Modbus 帧处理函数
ModbusStatus Modbus_Process_Set_24V(uint8_t *rxBuffer, uint16_t len, uint8_t *txBuffer, uint16_t *txLen)
{

    txBuffer[0] = 0x01;
    txBuffer[1] = 0x10;
    txBuffer[2] = 0x00;
    txBuffer[3] = 0x09;
    txBuffer[4] = 0x00;
    txBuffer[5] = 0x05;
    txBuffer[6] = 0x0A;
    txBuffer[7] = 0x00;
    txBuffer[8] = 0x03;
    txBuffer[9] = 0x00;
    txBuffer[10] = 0x00;
    txBuffer[11] = 0x41;
    txBuffer[12] = 0xC0;
    txBuffer[13] = 0x00;
    txBuffer[14] = 0x00;
    txBuffer[15] = 0x41;
    txBuffer[16] = 0x70;
    txBuffer[17] = 0x00;
    txBuffer[18] = 0x00;

    // 增加校验位
    uint16_t response_crc = CRC16_Calculate(txBuffer, 19);
    txBuffer[19] = response_crc & 0xFF;
    txBuffer[20] = response_crc >> 8;

    // 设置数据长度
    *txLen = 19 + 2; // 数据长度 + CRC

    return MODBUS_OK;
}

// Modbus 帧处理函数
ModbusStatus Modbus_Process_Set_12V(uint8_t *rxBuffer, uint16_t len, uint8_t *txBuffer, uint16_t *txLen)
{

    txBuffer[0] = 0x01;
    txBuffer[1] = 0x10;
    txBuffer[2] = 0x00;
    txBuffer[3] = 0x09;
    txBuffer[4] = 0x00;
    txBuffer[5] = 0x05;
    txBuffer[6] = 0x0A;
    txBuffer[7] = 0x00;
    txBuffer[8] = 0x03;
    txBuffer[9] = 0x00;
    txBuffer[10] = 0x00;
    txBuffer[11] = 0x41;
    txBuffer[12] = 0x40;
    txBuffer[13] = 0x00;
    txBuffer[14] = 0x00;
    txBuffer[15] = 0x41;
    txBuffer[16] = 0x70;
    txBuffer[17] = 0x00;
    txBuffer[18] = 0x00;

    // 增加校验位
    uint16_t response_crc = CRC16_Calculate(txBuffer, 19);
    txBuffer[19] = response_crc & 0xFF;
    txBuffer[20] = response_crc >> 8;

    // 设置数据长度
    *txLen = 19 + 2; // 数据长度 + CRC

    return MODBUS_OK;
}