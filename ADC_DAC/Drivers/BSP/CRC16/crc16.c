#include "crc16.h"

#define POLY 0xA001

// Modbus CRC16 校验位计算，获取数据和长度
uint16_t CRC16_Calculate(const uint8_t *data, uint16_t length) {
    uint16_t crc = 0xFFFF;

    for (uint16_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x01) {
                crc = (crc >> 1) ^ POLY;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}
