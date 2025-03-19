#ifndef __PID_H
#define __PID_H

#include "stm32f4xx_hal.h"

// PID 结构体
typedef struct {
    float Kp;        // 比例系数
    float Ki;        // 积分系数
    float Kd;        // 微分系数
    float target;    // 目标值
    float integral;  // 积分项
    float prev_error;// 上一次误差
    float output;    // 输出值
    float max_output;// 最大输出限制
    float min_output;// 最小输出限制
} PID_TypeDef;

// 初始化 PID 控制器
void PID_Init(PID_TypeDef *pid, float Kp, float Ki, float Kd, float max_output, float min_output);

// 计算 PID 输出
float PID_Compute(PID_TypeDef *pid, float current_value);

#endif
