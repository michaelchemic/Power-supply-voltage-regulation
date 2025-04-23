#include "./BSP/ACS712/ACS712.h"
#include "./SYSTEM/sys/sys.h"
#include "./BSP/RS485/rs485.h"
#include "./BSP/Modbus_RTU/modbus.h"

// ��ACS712 ��⵽���� 3300��4096�� 0.5Aʱ��ֱ������Ŀ��ֵ48V��

/* 
��ѹ�������ձ�

3300 0.5A 2.65V
3350 1A  2.7V
3400 1.5A 2.75V
3450 2A 2.8V
3500 2.5A 2.85V
*/

uint16_t Value = 3200;
extern uint16_t ADC2_Value;

// �ⲿ����֡
extern uint8_t rxBuffer[21];
extern uint8_t txBuffer[21];
extern uint16_t txLen;

// �������κ���
void Current_Start()
{
    if (Modbus_Process_Set_12V(rxBuffer, sizeof(rxBuffer), txBuffer, &txLen) == MODBUS_OK)
    {                                  // �жϷ��������Ƿ�����
        rs485_send_data(txBuffer, 21); // Modbus�����õ�����ͨ��485����
        HAL_Delay(500);
    }
     if (Modbus_Process_Set_24V(rxBuffer, sizeof(rxBuffer), txBuffer, &txLen) == MODBUS_OK)
    {                                  // �жϷ��������Ƿ�����
        rs485_send_data(txBuffer, 21); // Modbus�����õ�����ͨ��485����
        HAL_Delay(2);
    }
     if (Modbus_Process_Set_36V(rxBuffer, sizeof(rxBuffer), txBuffer, &txLen) == MODBUS_OK)
    {                                  // �жϷ��������Ƿ�����
        rs485_send_data(txBuffer, 21); // Modbus�����õ�����ͨ��485����
        HAL_Delay(2);
    }
     if (Modbus_Process_Set_48V(rxBuffer, sizeof(rxBuffer), txBuffer, &txLen) == MODBUS_OK)
    {                                  // �жϷ��������Ƿ�����
        rs485_send_data(txBuffer, 21); // Modbus�����õ�����ͨ��485����
        HAL_Delay(2);
    }
     if (Modbus_Process_Set_72V(rxBuffer, sizeof(rxBuffer), txBuffer, &txLen) == MODBUS_OK)
    {                                  // �жϷ��������Ƿ�����
        rs485_send_data(txBuffer, 21); // Modbus�����õ�����ͨ��485����
        HAL_Delay(2);
    }
    
}

// �������κ���
void Current_Comparator()
{
     if (Modbus_Process_Set_72V(rxBuffer, sizeof(rxBuffer), txBuffer, &txLen) == MODBUS_OK)
    {                                  // �жϷ��������Ƿ�����
        rs485_send_data(txBuffer, 21); // Modbus�����õ�����ͨ��485����
        HAL_Delay(2);
    }
}
