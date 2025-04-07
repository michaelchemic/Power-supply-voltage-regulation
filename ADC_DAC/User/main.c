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

#define ADC_DMA_BUF_SIZE 50 /* ADC DMA采集 BUF大小, 应等于ADC通道数的整数倍 */

#define R1 210.0f // 分压电阻 R1
#define R2 10.0f  // 分压电阻 R2

uint16_t g_adc_dma_buf[ADC_DMA_BUF_SIZE]; /* ADC DMA BUF */

// ADC采集的负载电压设为全局变量
uint8_t bytes[4]; // 浮点数电压转换成4字节表示

uint16_t adc2_value; // 电流传感器

uint8_t rxBuffer[21]; // RX Buffer
uint8_t txBuffer[21]; // TX Buffer
uint16_t txLen = 0;   // 串口数据长度

extern uint8_t g_adc_dma_sta; /* DMA传输状态标志, 0,未完成; 1, 已完成 */

extern DAC_HandleTypeDef g_dac1_handle; /* 获取 DAC1 句柄 */

extern uint16_t Value; /* 电流传感器采集的电压值 */

int main(void)
{

    /* adc变量 */
    uint16_t i;
    uint16_t adc_value;
    uint32_t sum;

    /* 串口通信变量 */
    uint8_t len;
    uint16_t times = 0;

    /* 485变量 */
    uint8_t cnt = 0;     // 计数
    uint8_t rs485buf[5]; // 设置485 buffer缓冲

    UART_HandleTypeDef huart1; //获取串口1句柄

    HAL_Init();                         /* 初始化HAL库 */
    sys_stm32_clock_init(336, 8, 2, 7); /* 设置时钟,168Mhz */
    delay_init(168);                    /* 延时初始化 */
    usart_init(115200);                 /* 串口初始化为115200 */
    rs485_init(115200);                 /* 初始化RS485 */

    adc_dma_init((uint32_t)&g_adc_dma_buf); /* 初始化ADC DMA采集 */
    adc_dma_enable(ADC_DMA_BUF_SIZE);       /* 启动ADC DMA采集 */

    dac_init(1); //DAC初始化

    PID_TypeDef voltage_pid; // 创建 PID 结构体实例

    // 初始化 PID 参数 (Kp, Ki, Kd, 最大输出, 最小输出)
    // PID_Init(&voltage_pid, 1.5f, 0.8f, 0.1f, 4095.0f, 0.0f); // 1.5 0.8
    PID_Init(&voltage_pid, 1.5f, 0.8f, 0.1f, 4095.0f, 0.0f); // 1.5 0.8

    voltage_pid.target = 48.0f; //设置PID目标电压 48V

    Current_Start(); // 电压分段设置成48V
    
    HAL_Delay(5000);

    while (1)
    {

        if (g_adc_dma_sta == 1) // 判断标志位g_adc_dma_sta && g_adc2_dma_sta == 1
        {

            adc2_value = adc2_get_result_average(ADC2_ADCX_CHY, 100);

            /* 计算DMA 采集到的ADC数据的平均值 */
            sum = 0;

            for (i = 0; i < ADC_DMA_BUF_SIZE; i++) /* 累加平均 */
            {
                sum += g_adc_dma_buf[i];
            }

            adc_value = sum / ADC_DMA_BUF_SIZE; /* 取平均值 */
            
            //  循环中调用 PID 计算
            // float load_voltage = adc_voltage * ((R1 + R2) / R2);//分压电阻计算公式
            float adc_voltage = (float)adc_value * (3.3 / 4096); // 获取ADC实际电压值

            // 转换为实际负载电压
            float load_voltage = adc_voltage * ((R1 + R2) / R2);

            uint16_t dac_output = (uint16_t)PID_Compute(&voltage_pid, load_voltage); // 设置 DAC 输出

            uint16_t dac_voltage = dac_output;

            HAL_DAC_SetValue(&g_dac1_handle, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dac_voltage);

            float Real_DAC_voltage = (3.3 / 4096) * dac_voltage;

            float ACDC_voltage = Real_DAC_voltage * (float)60.606; // 这里通过某种计算方式把dac 0-3.3V范围映射到0-200V范围

            float_to_ieee754(ACDC_voltage, bytes); // 浮点数电压转换四字节函数

            if (adc2_value <= Value)
            {
                Current_Comparator(); // 电压设置成48V
                dac_voltage=0;
            }
            else 
            {
                Modbus_Process(rxBuffer, sizeof(rxBuffer), txBuffer, &txLen);
           
                    // 判断发送数据是否正常
                    rs485_send_data(txBuffer, 21); // Modbus解析好的数据通过485传出
                    HAL_Delay(5);
   
            }
            g_adc_dma_sta = 0; /* 清除DMA采集完成状态标志 */

            adc_dma_enable(ADC_DMA_BUF_SIZE); /* 启动下一次ADC DMA采集 */

            printf("%d,%d,%d\n", dac_output, adc_value, adc2_value); // DAC和ADC值串口打印，上位机观察曲线。
            //printf("%d,%d\n", dac_output, adc_value); // DAC和ADC值串口打印，上位机观察曲线。

            while (__HAL_UART_GET_FLAG(&g_uart1_handle, UART_FLAG_TC) != SET)
                ; /* 等待发送结束 */
            g_usart_rx_sta = 0;
        }
    }
}
