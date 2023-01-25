#ifndef INC_CALLBACKS_H_
#define INC_CALLBACKS_H_

#include "stm32f7xx_hal.h"


void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc);

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

#endif /* INC_CALLBACKS_H_ */
