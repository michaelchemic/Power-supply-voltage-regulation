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

#define ADC_DMA_BUF_SIZE 50  /* ADC DMA�ɼ� BUF��С, Ӧ����ADCͨ������������ */
#define ADC2_DMA_BUF_SIZE 50 /* ADC DMA�ɼ� BUF��С, Ӧ����ADCͨ������������ */

#define R1 210.0f // ��ѹ���� R1
#define R2 10.0f  // ��ѹ���� R2

uint16_t g_adc_dma_buf[ADC_DMA_BUF_SIZE];   /* ADC DMA BUF */
uint16_t g_adc2_dma_buf[ADC2_DMA_BUF_SIZE]; /* ADC DMA BUF */
// ADC�ɼ��ĸ��ص�ѹ��Ϊȫ�ֱ���
uint8_t bytes[4]; // ��������ѹת����4�ֽڱ�ʾ

uint16_t ADC2_Value; // ����������

uint8_t rxBuffer[21];
uint8_t txBuffer[21];
uint16_t txLen = 0;

extern uint8_t g_adc_dma_sta;  /* DMA����״̬��־, 0,δ���; 1, ����� */
extern uint8_t g_adc2_dma_sta; /* DMA����״̬��־, 0,δ���; 1, ����� */
extern DAC_HandleTypeDef g_dac1_handle;
extern DAC_HandleTypeDef g_dac2_handle;

int main(void)
{

    /* adc���� */
    uint16_t i;
    uint16_t adc_value;
    uint16_t adc_value1;
    uint32_t sum;
    uint32_t sum1;
    /* ����ͨ�ű��� */
    uint8_t len;
    uint16_t times = 0;
    // uint16_t dacval = 4095; /* DACֵ����ֵ */
    /* 485���� */
    uint8_t cnt = 0;     // ����
    uint8_t rs485buf[5]; // ����buffer����

    UART_HandleTypeDef huart1;

    HAL_Init();                         /* ��ʼ��HAL�� */
    sys_stm32_clock_init(336, 8, 2, 7); /* ����ʱ��,168Mhz */
    delay_init(168);                    /* ��ʱ��ʼ�� */
    usart_init(115200);                 /* ���ڳ�ʼ��Ϊ115200 */
    // led_init();                             /* ��ʼ��LED */
    rs485_init(115200); /* ��ʼ��RS485 */

    adc_dma_init((uint32_t)&g_adc_dma_buf); /* ��ʼ��ADC DMA�ɼ� */
    adc_dma_enable(ADC_DMA_BUF_SIZE);       /* ����ADC DMA�ɼ� */

    adc2_dma_init((uint32_t)&g_adc2_dma_buf); /* ��ʼ��ADC DMA�ɼ� */
    adc2_dma_enable(ADC2_DMA_BUF_SIZE);       /* ����ADC DMA�ɼ� */

    dac_init(1);

    // ���� PID �ṹ��ʵ��
    PID_TypeDef voltage_pid;

    // ��ʼ�� PID ���� (Kp, Ki, Kd, ������, ��С���)
    PID_Init(&voltage_pid, 1.5f, 0.8f, 0.1f, 4095.0f, 0.0f); // 1.5 0.8

    // ����Ŀ���ѹ
    voltage_pid.target = 48.0f; // Ŀ���ѹ 48V

    while (1)
    {

        //          if (Modbus_Receive(rxBuffer, sizeof(rxBuffer)) == MODBUS_OK) {    //���жϽ��������Ƿ�����
        //            if (Modbus_Process(rxBuffer, sizeof(rxBuffer), txBuffer, &txLen) == MODBUS_OK) { //�жϷ��������Ƿ�����
        //                HAL_UART_Transmit(&huart1, txBuffer, txLen, 1000); //��������
        //            }
        //        }

        //         for (i = 0; i < 5; i++)
        //            {
        //                rs485buf[i] = cnt + i;      /* ��䷢�ͻ����� */
        //                lcd_show_xnum(30 + i * 32, 170, rs485buf[i], 3, 16, 0X80, BLUE);    /* ��ʾ���� */
        //            }
        //         rs485_send_data(rs485buf, 5);   /* ����5���ֽ� */
        //         HAL_Delay(500);

        if ( 1) // �жϱ�־λg_adc_dma_sta && g_adc2_dma_sta == 1
        {

            ADC2_Value = adc2_get_result_average(ADC2_ADCX_CHY, 3); // ADC2 ��ȡ���ԣ�����������

            /* ����DMA �ɼ�����ADC���ݵ�ƽ��ֵ */
            sum = 0;
            sum1 = 0;

            for (i = 0; i < ADC_DMA_BUF_SIZE; i++) /* �ۼ�ƽ�� */
            {
                sum += g_adc_dma_buf[i];
            }

            for (i = 0; i < ADC2_DMA_BUF_SIZE; i++) /* �ۼ�ƽ�� */
            {
                sum1 += g_adc2_dma_buf[i];
            }

            adc_value = sum / ADC_DMA_BUF_SIZE; /* ȡƽ��ֵ */

            adc_value1 = sum1 / ADC2_DMA_BUF_SIZE; /* ȡƽ��ֵ */

            // HAL_DAC_SetValue(&g_dac1_handle, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dacval);   //���Դ���
            //  ���� DAC ���
            //  ѭ���е��� PID ����
            // float load_voltage = adc_voltage * ((R1 + R2) / R2);//��ѹ������㹫ʽ
            float adc_voltage = (float)adc_value * (3.3 / 4096); // ��ȡADCʵ�ʵ�ѹֵ
            // ת��Ϊʵ�ʸ��ص�ѹ
            float load_voltage = adc_voltage * ((R1 + R2) / R2);

            uint16_t dac_output = (uint16_t)PID_Compute(&voltage_pid, load_voltage);

            uint16_t dac_voltage = dac_output;

            HAL_DAC_SetValue(&g_dac1_handle, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dac_voltage);

            float Real_DAC_voltage = (3.3 / 4096) * dac_voltage;

            float ACDC_voltage = Real_DAC_voltage * (float)60.606; // ����ͨ��ĳ�ּ��㷽ʽ��dac 0-3.3V��Χӳ�䵽0-200V��Χ

            float_to_ieee754(ACDC_voltage, bytes); // ��������ѹת�����ֽں���

            if (Modbus_Process(rxBuffer, sizeof(rxBuffer), txBuffer, &txLen) == MODBUS_OK)
            {                                  // �жϷ��������Ƿ�����
                rs485_send_data(txBuffer, 21); // Modbus�����õ�����ͨ��485����
                HAL_Delay(5);
            }
            else
            {
                Current_comparator(); // ��ѹ���ó�48V
            }

            g_adc_dma_sta = 0; /* ���DMA�ɼ����״̬��־ */
            g_adc2_dma_sta =0;
            adc_dma_enable(ADC_DMA_BUF_SIZE);   /* ������һ��ADC DMA�ɼ� */
            adc2_dma_enable(ADC2_DMA_BUF_SIZE); /* ������һ��ADC DMA�ɼ� */

            printf("%d,%d,%d,%d\n", dac_output, adc_value, ADC2_Value,adc_value1); // DAC��ADCֵ���ڴ�ӡ����λ���۲����ߡ�

            while (__HAL_UART_GET_FLAG(&g_uart1_handle, UART_FLAG_TC) != SET)
                ; /* �ȴ����ͽ��� */
            // printf("\r\n\r\n");             /* ���뻻�� */
            g_usart_rx_sta = 0;
        }
    }
}
