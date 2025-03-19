#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/usart/usart.h"
#include "./SYSTEM/delay/delay.h"
#include "./BSP/LED/led.h"
#include "./BSP/LCD/lcd.h"
#include "./BSP/ADC/adc.h"
#include "./BSP/DAC/dac.h"
#include "./BSP/PID/pid.h"

#define ADC_DMA_BUF_SIZE        50          /* ADC DMA�ɼ� BUF��С, Ӧ����ADCͨ������������ */
#define R1 210.0f  // ��ѹ���� R1
#define R2 10.0f   // ��ѹ���� R2

uint16_t g_adc_dma_buf[ADC_DMA_BUF_SIZE];   /* ADC DMA BUF */
extern uint8_t g_adc_dma_sta;               /* DMA����״̬��־, 0,δ���; 1, ����� */
extern DAC_HandleTypeDef g_dac1_handle;

int main(void)
{
    /* adc���� */
    uint16_t i;
    uint16_t adc_value;
    uint32_t sum;
    /* ����ͨ�ű��� */
    uint8_t len;
    uint16_t times = 0;
    //uint16_t dacval = 4095; /* DACֵ����ֵ */

    HAL_Init();                             /* ��ʼ��HAL�� */
    sys_stm32_clock_init(336, 8, 2, 7);     /* ����ʱ��,168Mhz */
    delay_init(168);                        /* ��ʱ��ʼ�� */
    usart_init(115200);                     /* ���ڳ�ʼ��Ϊ115200 */
    //led_init();                             /* ��ʼ��LED */

    adc_dma_init((uint32_t)&g_adc_dma_buf); /* ��ʼ��ADC DMA�ɼ� */
    adc_dma_enable(ADC_DMA_BUF_SIZE);       /* ����ADC DMA�ɼ� */
    dac_init(1);  
    
    // ���� PID �ṹ��ʵ��
    PID_TypeDef voltage_pid;

    // ��ʼ�� PID ���� (Kp, Ki, Kd, ������, ��С���)
    PID_Init(&voltage_pid, 1.5f, 0.8f, 0.1f, 4095.0f, 0.0f);//1.5 0.8

    // ����Ŀ���ѹ
    voltage_pid.target = 48.0f;  // Ŀ���ѹ 48V
    
    while (1)
    {
        if (g_adc_dma_sta == 1) //�жϱ�־λ
        {
            /* ����DMA �ɼ�����ADC���ݵ�ƽ��ֵ */
            sum = 0;

            for (i = 0; i < ADC_DMA_BUF_SIZE; i++)  /* �ۼ�ƽ�� */
            {
                sum += g_adc_dma_buf[i];
            }

            adc_value = sum / ADC_DMA_BUF_SIZE;          /* ȡƽ��ֵ */
            
            //HAL_DAC_SetValue(&g_dac1_handle, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dacval);   //���Դ���
            // ���� DAC ���
            // ѭ���е��� PID ����
            //float load_voltage = adc_voltage * ((R1 + R2) / R2);//��ѹ������㹫ʽ 
            float adc_voltage = (float)adc_value * (3.3 / 4096);  // ��ȡADCʵ�ʵ�ѹֵ 
            // ת��Ϊʵ�ʸ��ص�ѹ
            float load_voltage = adc_voltage * ((R1 + R2) / R2);  
            uint16_t dac_output = (uint16_t)PID_Compute(&voltage_pid, load_voltage);
            
            uint16_t dac_voltage=dac_output;
                      
            HAL_DAC_SetValue(&g_dac1_handle, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dac_voltage);
            
            g_adc_dma_sta = 0;                                  /* ���DMA�ɼ����״̬��־ */
            adc_dma_enable(ADC_DMA_BUF_SIZE);                   /* ������һ��ADC DMA�ɼ� */
            
            HAL_Delay(5);
     
            printf("%d\n",dac_output);
            while(__HAL_UART_GET_FLAG(&g_uart1_handle,UART_FLAG_TC)!=SET);           /* �ȴ����ͽ��� */
            //printf("\r\n\r\n");             /* ���뻻�� */
            g_usart_rx_sta = 0;
        
        }

    }
}








