#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/usart/usart.h"
#include "./SYSTEM/delay/delay.h"
#include "./BSP/LED/led.h"
#include "./BSP/LCD/lcd.h"
#include "./BSP/ADC/adc.h"
#include "./BSP/DAC/dac.h"
#include "./BSP/PID/pid.h"

#define ADC_DMA_BUF_SIZE        50          /* ADC DMA采集 BUF大小, 应等于ADC通道数的整数倍 */
#define R1 210.0f  // 分压电阻 R1
#define R2 10.0f   // 分压电阻 R2

uint16_t g_adc_dma_buf[ADC_DMA_BUF_SIZE];   /* ADC DMA BUF */
extern uint8_t g_adc_dma_sta;               /* DMA传输状态标志, 0,未完成; 1, 已完成 */
extern DAC_HandleTypeDef g_dac1_handle;

int main(void)
{
    /* adc变量 */
    uint16_t i;
    uint16_t adc_value;
    uint32_t sum;
    /* 串口通信变量 */
    uint8_t len;
    uint16_t times = 0;
    //uint16_t dacval = 4095; /* DAC值测试值 */

    HAL_Init();                             /* 初始化HAL库 */
    sys_stm32_clock_init(336, 8, 2, 7);     /* 设置时钟,168Mhz */
    delay_init(168);                        /* 延时初始化 */
    usart_init(115200);                     /* 串口初始化为115200 */
    //led_init();                             /* 初始化LED */

    adc_dma_init((uint32_t)&g_adc_dma_buf); /* 初始化ADC DMA采集 */
    adc_dma_enable(ADC_DMA_BUF_SIZE);       /* 启动ADC DMA采集 */
    dac_init(1);  
    
    // 创建 PID 结构体实例
    PID_TypeDef voltage_pid;

    // 初始化 PID 参数 (Kp, Ki, Kd, 最大输出, 最小输出)
    PID_Init(&voltage_pid, 1.5f, 0.8f, 0.1f, 4095.0f, 0.0f);//1.5 0.8

    // 设置目标电压
    voltage_pid.target = 48.0f;  // 目标电压 48V
    
    while (1)
    {
        if (g_adc_dma_sta == 1) //判断标志位
        {
            /* 计算DMA 采集到的ADC数据的平均值 */
            sum = 0;

            for (i = 0; i < ADC_DMA_BUF_SIZE; i++)  /* 累加平均 */
            {
                sum += g_adc_dma_buf[i];
            }

            adc_value = sum / ADC_DMA_BUF_SIZE;          /* 取平均值 */
            
            //HAL_DAC_SetValue(&g_dac1_handle, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dacval);   //测试代码
            // 设置 DAC 输出
            // 循环中调用 PID 计算
            //float load_voltage = adc_voltage * ((R1 + R2) / R2);//分压电阻计算公式 
            float adc_voltage = (float)adc_value * (3.3 / 4096);  // 获取ADC实际电压值 
            // 转换为实际负载电压
            float load_voltage = adc_voltage * ((R1 + R2) / R2);  
            uint16_t dac_output = (uint16_t)PID_Compute(&voltage_pid, load_voltage);
            
            uint16_t dac_voltage=dac_output;
                      
            HAL_DAC_SetValue(&g_dac1_handle, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dac_voltage);
            
            g_adc_dma_sta = 0;                                  /* 清除DMA采集完成状态标志 */
            adc_dma_enable(ADC_DMA_BUF_SIZE);                   /* 启动下一次ADC DMA采集 */
            
            HAL_Delay(5);
     
            printf("%d\n",dac_output);
            while(__HAL_UART_GET_FLAG(&g_uart1_handle,UART_FLAG_TC)!=SET);           /* 等待发送结束 */
            //printf("\r\n\r\n");             /* 插入换行 */
            g_usart_rx_sta = 0;
        
        }

    }
}








