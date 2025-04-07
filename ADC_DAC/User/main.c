#include "./stdbool.h"
#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/usart/usart.h"
#include "./SYSTEM/delay/delay.h"
#include "./BSP/LED/led.h"
#include "./BSP/LCD/lcd.h"
#include "./BSP/ADC/adc.h"
#include "./BSP/DAC/dac.h"
#include "./BSP/PID/pid.h"
#include "./BSP/RS485/rs485.h"
#include "./BSP/Modbus_RTU/modbus.h"
#include "./BSP/IEEE754/IEEE754.h"
#include "./BSP/ACS712/ACS712.h"

#define ADC_DMA_BUF_SIZE 50 /* ADC DMA�ɼ� BUF��С, Ӧ����ADCͨ������������ */

#define R1 210.0f // ��ѹ���� R1
#define R2 10.0f  // ��ѹ���� R2

uint16_t g_adc_dma_buf[ADC_DMA_BUF_SIZE]; /* ADC DMA BUF */

// ADC�ɼ��ĸ��ص�ѹ��Ϊȫ�ֱ���
uint8_t bytes[4]; // ��������ѹת����4�ֽڱ�ʾ

uint16_t adc2_value; // ����������

uint8_t rxBuffer[21]; // RX Buffer
uint8_t txBuffer[21]; // TX Buffer
uint16_t txLen = 0;   // �������ݳ���

extern uint8_t g_adc_dma_sta; /* DMA����״̬��־, 0,δ���; 1, ����� */

extern DAC_HandleTypeDef g_dac1_handle; /* ��ȡ DAC1 ��� */

extern uint16_t Value; /* �����������ɼ��ĵ�ѹֵ */

int main(void)
{

    /* adc���� */
    uint16_t i;
    uint16_t adc_value;
    uint32_t sum;

    /* ����ͨ�ű��� */
    uint8_t len;
    uint16_t times = 0;

    /* 485���� */
    uint8_t cnt = 0;     // ����
    uint8_t rs485buf[5]; // ����485 buffer����

    UART_HandleTypeDef huart1; //��ȡ����1���

    HAL_Init();                         /* ��ʼ��HAL�� */
    sys_stm32_clock_init(336, 8, 2, 7); /* ����ʱ��,168Mhz */
    delay_init(168);                    /* ��ʱ��ʼ�� */
    usart_init(115200);                 /* ���ڳ�ʼ��Ϊ115200 */
    rs485_init(115200);                 /* ��ʼ��RS485 */

    adc_dma_init((uint32_t)&g_adc_dma_buf); /* ��ʼ��ADC DMA�ɼ� */
    adc_dma_enable(ADC_DMA_BUF_SIZE);       /* ����ADC DMA�ɼ� */

    dac_init(1); //DAC��ʼ��

    PID_TypeDef voltage_pid; // ���� PID �ṹ��ʵ��

    // ��ʼ�� PID ���� (Kp, Ki, Kd, ������, ��С���)
    // PID_Init(&voltage_pid, 1.5f, 0.8f, 0.1f, 4095.0f, 0.0f); // 1.5 0.8
    PID_Init(&voltage_pid, 1.5f, 0.8f, 0.1f, 4095.0f, 0.0f); // 1.5 0.8

    voltage_pid.target = 48.0f; //����PIDĿ���ѹ 48V

    Current_Start(); // ��ѹ�ֶ����ó�48V
    
    HAL_Delay(5000);

    while (1)
    {

        if (g_adc_dma_sta == 1) // �жϱ�־λg_adc_dma_sta && g_adc2_dma_sta == 1
        {

            adc2_value = adc2_get_result_average(ADC2_ADCX_CHY, 100);

            /* ����DMA �ɼ�����ADC���ݵ�ƽ��ֵ */
            sum = 0;

            for (i = 0; i < ADC_DMA_BUF_SIZE; i++) /* �ۼ�ƽ�� */
            {
                sum += g_adc_dma_buf[i];
            }

            adc_value = sum / ADC_DMA_BUF_SIZE; /* ȡƽ��ֵ */
            
            //  ѭ���е��� PID ����
            // float load_voltage = adc_voltage * ((R1 + R2) / R2);//��ѹ������㹫ʽ
            float adc_voltage = (float)adc_value * (3.3 / 4096); // ��ȡADCʵ�ʵ�ѹֵ

            // ת��Ϊʵ�ʸ��ص�ѹ
            float load_voltage = adc_voltage * ((R1 + R2) / R2);

            uint16_t dac_output = (uint16_t)PID_Compute(&voltage_pid, load_voltage); // ���� DAC ���

            uint16_t dac_voltage = dac_output;

            HAL_DAC_SetValue(&g_dac1_handle, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dac_voltage);

            float Real_DAC_voltage = (3.3 / 4096) * dac_voltage;

            float ACDC_voltage = Real_DAC_voltage * (float)60.606; // ����ͨ��ĳ�ּ��㷽ʽ��dac 0-3.3V��Χӳ�䵽0-200V��Χ

            float_to_ieee754(ACDC_voltage, bytes); // ��������ѹת�����ֽں���

            if (adc2_value <= Value)
            {
                Current_Comparator(); // ��ѹ���ó�48V
                dac_voltage=0;
            }
            else 
            {
                Modbus_Process(rxBuffer, sizeof(rxBuffer), txBuffer, &txLen);
           
                    // �жϷ��������Ƿ�����
                    rs485_send_data(txBuffer, 21); // Modbus�����õ�����ͨ��485����
                    HAL_Delay(5);
   
            }
            g_adc_dma_sta = 0; /* ���DMA�ɼ����״̬��־ */

            adc_dma_enable(ADC_DMA_BUF_SIZE); /* ������һ��ADC DMA�ɼ� */

            printf("%d,%d,%d\n", dac_output, adc_value, adc2_value); // DAC��ADCֵ���ڴ�ӡ����λ���۲����ߡ�
            //printf("%d,%d\n", dac_output, adc_value); // DAC��ADCֵ���ڴ�ӡ����λ���۲����ߡ�

            while (__HAL_UART_GET_FLAG(&g_uart1_handle, UART_FLAG_TC) != SET)
                ; /* �ȴ����ͽ��� */
            g_usart_rx_sta = 0;
        }
    }
}
