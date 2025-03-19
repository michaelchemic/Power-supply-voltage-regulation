/*
 * 文件名: pid.c
 * 类型：debug版本测试代码
 * 作者: MichaelKing
 * 日期: 2024-06-26
 * 版本: 1.0
 * 描述: 电机的速度环，距离环，电流环的pid控制器。
 *
 * 历史记录:
 * 版本 1.0 - 2024-06-26 - MichaelKing
 *   初始版本。
 */
#include "pid.h"

/**
 * @brief  初始化 PID 结构体
 * @param  pid: PID 控制结构体
 * @param  Kp, Ki, Kd: PID 参数
 * @param  max_output, min_output: 输出限幅
 */
void PID_Init(PID_TypeDef *pid, float Kp, float Ki, float Kd, float max_output, float min_output) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->output = 0.0f;
    pid->max_output = max_output;
    pid->min_output = min_output;
}

/**
 * @brief  计算 PID 输出
 * @param  pid: PID 控制结构体
 * @param  current_value: 当前测量值
 * @return 计算后的输出值
 */
//float PID_Compute(PID_TypeDef *pid, float current_value) {
//    float error = pid->target - current_value; // 计算误差
//    pid->integral += error; // 积分
//    float derivative = error - pid->prev_error; // 微分
//    pid->output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative; // 计算 PID 输出

//    // 限制输出范围
//    if (pid->output > pid->max_output) pid->output = pid->max_output;
//    if (pid->output < pid->min_output) pid->output = pid->min_output;

//    pid->prev_error = error; // 记录本次误差

//    return pid->output;

//}
float PID_Compute(PID_TypeDef *pid, float current_value) {
    float error = pid->target - current_value;       // 计算误差
    pid->integral += error;                          // 积分

    // 限制积分项大小，防止积分饱和
    if (pid->integral > pid->max_output / pid->Ki) {
        pid->integral = pid->max_output / pid->Ki;
    }
    if (pid->integral < pid->min_output / pid->Ki) {
        pid->integral = pid->min_output / pid->Ki;
    }

    float derivative = error - pid->prev_error;       // 微分
    pid->output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;

    // 限制输出范围
    if (pid->output > pid->max_output) pid->output = pid->max_output;
    if (pid->output < pid->min_output) pid->output = pid->min_output;

    pid->prev_error = error;                         // 记录本次误差
    return pid->output;
}


