// LED_MATRIX.h
#ifndef LED_MATRIX_H
#define LED_MATRIX_H

#include <stdint.h>

extern const uint8_t data[];
extern const uint8_t data2[];
extern const uint8_t data3[];

void matrixInit(void);
void pattern1(void);
void pattern2(void);
void pattern3(void);
#endif // LED_MATRIX_H
