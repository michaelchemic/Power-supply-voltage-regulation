#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <IEEE754.h>

void float_to_ieee754(float value, uint8_t *bytes) {
    // 将浮点数按 IEEE 754 格式存储为 4 字节
    memcpy(bytes, &value, sizeof(float));
}

    // float num = 60.00f;              // 浮点数
    // uint8_t bytes[4];                // 存储4字节表示

    // 打印结果
    // printf("浮点数: %.2f\n", num);
    // printf("IEEE 754 格式: %02X %02X %02X %02X\n", bytes[3], bytes[2], bytes[1], bytes[0]);


