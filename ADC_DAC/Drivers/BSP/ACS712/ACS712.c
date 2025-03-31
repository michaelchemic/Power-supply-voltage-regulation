#include "./BSP/ACS712/ACS712.h"
#include "./SYSTEM/sys/sys.h"
#include "./BSP/RS485/rs485.h"
#include "./BSP/Modbus_RTU/modbus.h"

// ��ACS712 ��⵽���� 3300��4096�� 0.5Aʱ��ֱ������Ŀ��ֵ48V��
/* ��ѹ�������ձ�    
3300 0.5A 2.65V
3350 1A  2.7V
3400 1.5A 2.75V
3450 2A 2.8V
*/

uint16_t Value = 3300;
extern uint16_t ADC2_Value;

// �ⲿ����֡
extern uint8_t rxBuffer[21];
extern uint8_t txBuffer[21];
extern uint16_t txLen;

// �������κ���
void Current_comparator()
{
    if (ADC2_Value <= Value)
    { // ���ڱȽ�ֵ 3300 ���õ�Դ��48V
        if (Modbus_Process_SetZero(rxBuffer, sizeof(rxBuffer), txBuffer, &txLen) == MODBUS_OK)
        {                                  // �жϷ��������Ƿ�����
            rs485_send_data(txBuffer, 21); // Modbus�����õ�����ͨ��485����
            HAL_Delay(5);
        }
    }
}
