#ifndef __MODBUS_H
#define __MODBUS_H

#include "stm32f4xx_hal.h"
#include "crc16.h"

// Modbus 配置
#define MODBUS_SLAVE_ID     0x01    // 从机默认地址01~255
#define MODBUS_MAX_DATA     128     // 最大数据长度

// 功能码定义
#define MODBUS_FUNC_READ    0x03    // 读取保持寄存器
#define MODBUS_FUNC_WRITE   0x06    // 写单个寄存器
#define MODBUS_FUNC_READ_WRITE 0x10 // R/W读写寄存器

// Modbus状态
typedef enum {
    MODBUS_OK=0,
    MODBUS_ERROR,
    MODBUS_TIMEOUT
} ModbusStatus;

// Modbus数据结构
typedef struct {
    uint8_t address;                 // 从机地址
    uint8_t function;                // 功能码，对应手册命令
    uint16_t startAddr;              // 起始寄存器地址，对应手册寄存器地址
    uint16_t regCount;               // 寄存器数量，对应手册寄存器值
    uint16_t crc;                    // CRC校验，电源无校验位。
    uint8_t data[MODBUS_MAX_DATA];   // 8位数据位
} ModbusFrame;

void Modbus_Init(UART_HandleTypeDef *huart);    //端口初始化
ModbusStatus Modbus_Receive(uint8_t *rxBuffer, uint16_t len);   //Modbus接收
ModbusStatus Modbus_Process(uint8_t *rxBuffer, uint16_t len, uint8_t *txBuffer, uint16_t *txLen); //协议处理

ModbusStatus Modbus_Process_SetZero(uint8_t *rxBuffer, uint16_t len, uint8_t *txBuffer, uint16_t *txLen); //协议处理

#endif
