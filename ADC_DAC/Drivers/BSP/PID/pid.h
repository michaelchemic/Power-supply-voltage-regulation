#ifndef __PID_H
#define __PID_H

#include "stm32f4xx_hal.h"

// PID �ṹ��
typedef struct {
    float Kp;        // ����ϵ��
    float Ki;        // ����ϵ��
    float Kd;        // ΢��ϵ��
    float target;    // Ŀ��ֵ
    float integral;  // ������
    float prev_error;// ��һ�����
    float output;    // ���ֵ
    float max_output;// ����������
    float min_output;// ��С�������
} PID_TypeDef;

// ��ʼ�� PID ������
void PID_Init(PID_TypeDef *pid, float Kp, float Ki, float Kd, float max_output, float min_output);

// ���� PID ���
float PID_Compute(PID_TypeDef *pid, float current_value);

#endif
