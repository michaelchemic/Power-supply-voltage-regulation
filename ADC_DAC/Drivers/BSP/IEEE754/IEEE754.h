#ifndef __IEEE754_H
#define __IEEE754_H
#include <stdio.h>
#include <stdint.h>
#include <string.h>

void float_to_ieee754(float value, uint8_t *bytes);

#endif