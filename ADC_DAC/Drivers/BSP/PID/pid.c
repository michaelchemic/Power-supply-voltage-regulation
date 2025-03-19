/*
 * �ļ���: pid.c
 * ���ͣ�debug�汾���Դ���
 * ����: MichaelKing
 * ����: 2024-06-26
 * �汾: 1.0
 * ����: ������ٶȻ������뻷����������pid��������
 *
 * ��ʷ��¼:
 * �汾 1.0 - 2024-06-26 - MichaelKing
 *   ��ʼ�汾��
 */
#include "pid.h"

/**
 * @brief  ��ʼ�� PID �ṹ��
 * @param  pid: PID ���ƽṹ��
 * @param  Kp, Ki, Kd: PID ����
 * @param  max_output, min_output: ����޷�
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
 * @brief  ���� PID ���
 * @param  pid: PID ���ƽṹ��
 * @param  current_value: ��ǰ����ֵ
 * @return ���������ֵ
 */
//float PID_Compute(PID_TypeDef *pid, float current_value) {
//    float error = pid->target - current_value; // �������
//    pid->integral += error; // ����
//    float derivative = error - pid->prev_error; // ΢��
//    pid->output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative; // ���� PID ���

//    // ���������Χ
//    if (pid->output > pid->max_output) pid->output = pid->max_output;
//    if (pid->output < pid->min_output) pid->output = pid->min_output;

//    pid->prev_error = error; // ��¼�������

//    return pid->output;

//}
float PID_Compute(PID_TypeDef *pid, float current_value) {
    float error = pid->target - current_value;       // �������
    pid->integral += error;                          // ����

    // ���ƻ������С����ֹ���ֱ���
    if (pid->integral > pid->max_output / pid->Ki) {
        pid->integral = pid->max_output / pid->Ki;
    }
    if (pid->integral < pid->min_output / pid->Ki) {
        pid->integral = pid->min_output / pid->Ki;
    }

    float derivative = error - pid->prev_error;       // ΢��
    pid->output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;

    // ���������Χ
    if (pid->output > pid->max_output) pid->output = pid->max_output;
    if (pid->output < pid->min_output) pid->output = pid->min_output;

    pid->prev_error = error;                         // ��¼�������
    return pid->output;
}


