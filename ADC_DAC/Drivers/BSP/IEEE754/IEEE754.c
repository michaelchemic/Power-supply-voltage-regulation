#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <IEEE754.h>

void float_to_ieee754(float value, uint8_t *bytes) {
    // ���������� IEEE 754 ��ʽ�洢Ϊ 4 �ֽ�
    memcpy(bytes, &value, sizeof(float));
}

    // float num = 60.00f;              // ������
    // uint8_t bytes[4];                // �洢4�ֽڱ�ʾ

    // ��ӡ���
    // printf("������: %.2f\n", num);
    // printf("IEEE 754 ��ʽ: %02X %02X %02X %02X\n", bytes[3], bytes[2], bytes[1], bytes[0]);


