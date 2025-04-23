#include "./BSP/ACS712/ACS712.h"
#include "./SYSTEM/sys/sys.h"
#include "./BSP/RS485/rs485.h"
#include "./BSP/Modbus_RTU/modbus.h"

// 当ACS712 检测到电流 3300（4096） 0.5A时，直接设置目标值48V。

/* 
电压电流对照表

3300 0.5A 2.65V
3350 1A  2.7V
3400 1.5A 2.75V
3450 2A 2.8V
3500 2.5A 2.85V
*/

uint16_t Value = 3200;
extern uint16_t ADC2_Value;

// 外部缓冲帧
extern uint8_t rxBuffer[21];
extern uint8_t txBuffer[21];
extern uint16_t txLen;

// 电流调参函数
void Current_Start()
{
    if (Modbus_Process_Set_12V(rxBuffer, sizeof(rxBuffer), txBuffer, &txLen) == MODBUS_OK)
    {                                  // 判断发送数据是否正常
        rs485_send_data(txBuffer, 21); // Modbus解析好的数据通过485传出
        HAL_Delay(500);
    }
     if (Modbus_Process_Set_24V(rxBuffer, sizeof(rxBuffer), txBuffer, &txLen) == MODBUS_OK)
    {                                  // 判断发送数据是否正常
        rs485_send_data(txBuffer, 21); // Modbus解析好的数据通过485传出
        HAL_Delay(2);
    }
     if (Modbus_Process_Set_36V(rxBuffer, sizeof(rxBuffer), txBuffer, &txLen) == MODBUS_OK)
    {                                  // 判断发送数据是否正常
        rs485_send_data(txBuffer, 21); // Modbus解析好的数据通过485传出
        HAL_Delay(2);
    }
     if (Modbus_Process_Set_48V(rxBuffer, sizeof(rxBuffer), txBuffer, &txLen) == MODBUS_OK)
    {                                  // 判断发送数据是否正常
        rs485_send_data(txBuffer, 21); // Modbus解析好的数据通过485传出
        HAL_Delay(2);
    }
     if (Modbus_Process_Set_72V(rxBuffer, sizeof(rxBuffer), txBuffer, &txLen) == MODBUS_OK)
    {                                  // 判断发送数据是否正常
        rs485_send_data(txBuffer, 21); // Modbus解析好的数据通过485传出
        HAL_Delay(2);
    }
    
}

// 电流调参函数
void Current_Comparator()
{
     if (Modbus_Process_Set_72V(rxBuffer, sizeof(rxBuffer), txBuffer, &txLen) == MODBUS_OK)
    {                                  // 判断发送数据是否正常
        rs485_send_data(txBuffer, 21); // Modbus解析好的数据通过485传出
        HAL_Delay(2);
    }
}
