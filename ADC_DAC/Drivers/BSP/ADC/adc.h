#ifndef __ADC_H
#define __ADC_H

#include "./SYSTEM/sys/sys.h"


/******************************************************************************************/
/* ADC������ ���� */

#define ADC_ADCX_CHY_GPIO_PORT              GPIOA
#define ADC_ADCX_CHY_GPIO_PIN               GPIO_PIN_5
#define ADC_ADCX_CHY_GPIO_CLK_ENABLE()      do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0) /* PA��ʱ��ʹ�� */

#define ADC_ADCX                            ADC1 
#define ADC_ADCX_CHY                        ADC_CHANNEL_5       /* ͨ��Y,  0 <= Y <= 17 */ 
#define ADC_ADCX_CHY_CLK_ENABLE()           do{ __HAL_RCC_ADC1_CLK_ENABLE(); }while(0)  /* ADC1 ʱ��ʹ�� */

/* ADC2������ ���� */

#define ADC2_ADCX_CHY_GPIO_PORT              GPIOA
#define ADC2_ADCX_CHY_GPIO_PIN               GPIO_PIN_0
#define ADC2_ADCX_CHY_GPIO_CLK_ENABLE()      do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)         /* PA��ʱ��ʹ�� */

#define ADC2_ADCX                            ADC2 
#define ADC2_ADCX_CHY                        ADC_CHANNEL_0                                    /* ͨ��Y,  0 <= Y <= 17 */ 
#define ADC2_ADCX_CHY_CLK_ENABLE()           do{ __HAL_RCC_ADC2_CLK_ENABLE(); }while(0)          /* ADC1 ʱ��ʹ�� */


/* ADC��ͨ��/��ͨ�� DMA�ɼ� DMA��������� ����
 * ע��: �������ǵ�ͨ������ʹ������Ķ���.
 */
#define ADC_ADCX_DMASx                      DMA2_Stream4
#define ADC_ADCX_DMASx_Chanel               DMA_CHANNEL_0       /* ADC1_DMA����Դ */
#define ADC_ADCX_DMASx_IRQn                 DMA2_Stream4_IRQn
#define ADC_ADCX_DMASx_IRQHandler           DMA2_Stream4_IRQHandler

#define ADC_ADCX_DMASx_IS_TC()              ( DMA2->HISR & (1 << 5) )               /* �ж� DMA2_Stream4 ������ɱ�־, ����һ���ٺ�����ʽ,
                                                                                     * ���ܵ�����ʹ��, ֻ������if���������
                                                                                     */
#define ADC_ADCX_DMASx_CLR_TC()             do{ DMA2->HIFCR |= 1 << 5; }while(0)    /* ��� DMA2_Stream4 ������ɱ�־ */
/////////////////////////////////////////////////////////////////////////////////////
#define ADC2_ADCX_DMASx                      DMA2_Stream3
#define ADC2_ADCX_DMASx_Chanel               DMA_CHANNEL_1       /* ADC1_DMA����Դ */
#define ADC2_ADCX_DMASx_IRQn                 DMA2_Stream3_IRQn
#define ADC2_ADCX_DMASx_IRQHandler           DMA2_Stream3_IRQHandler

#define ADC2_ADCX_DMASx_IS_TC()              ( DMA2->LISR & (1 << 27) )              /* �ж� DMA2_Stream3 ������ɱ�־ */
#define ADC2_ADCX_DMASx_CLR_TC()             do{ DMA2->LIFCR |= (1 << 27); }while(0) /* ��� DMA2_Stream3 ������ɱ�־ */
/******************************************************************************************/

void adc_init(void);                    /* ADC��ʼ�� */
void adc_channel_set(ADC_HandleTypeDef *adc_handle, uint32_t ch, uint32_t rank, uint32_t stime); /* ADCͨ������ */
uint32_t adc_get_result(uint32_t ch);   /* ���ĳ��ͨ��ֵ */
uint32_t adc_get_result_average(uint32_t ch, uint8_t times);    /* �õ�ĳ��ͨ����������������ƽ��ֵ */
uint32_t adc2_get_result(uint32_t ch);   /* ���ĳ��ͨ��ֵ */
uint32_t adc2_get_result_average(uint32_t ch, uint8_t times);    /* �õ�ĳ��ͨ����������������ƽ��ֵ */

void adc_dma_init(uint32_t mar);        /* ADC DMA�ɼ���ʼ�� */
void adc2_dma_init(uint32_t mar);   
void adc_dma_enable( uint16_t ndtr);    /* ʹ��һ��ADC DMA�ɼ����� */
void adc2_dma_enable(uint16_t ndtr2);
#endif



