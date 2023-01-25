#ifndef INC_FFT_H_
#define INC_FFT_H_

#include "stm32f7xx_hal.h"

float abs_complex(float real, float imag);
void DisplayFFT();
void FFT(UART_HandleTypeDef *huart);

#endif /* INC_FFT_H_ */
