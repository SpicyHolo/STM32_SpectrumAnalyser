#ifndef INC_GLOBAL_VARIABLES_H_
#define INC_GLOBAL_VARIABLES_H_

#define ARM_MATH_CM7
#define __FPU_PRESENT             1U

// Includes
#include "arm_math.h"
#include <stdint.h>

// FFT States
enum buffer_states{FFT_BUFFER_CLEAR, FFT_BUFFER_HALF, FFT_BUFFER_FULL, FFT_DISPLAY};
extern uint8_t buffer_state;

// Display States
enum display_states{DISPLAY_MANY, DISPLAY_FEW, DISPLAY_COW};
extern uint8_t display_state;

// Declare Arrays
extern uint32_t adc_buffer[8192];
extern float32_t fft_input_buffer[2048];
extern float32_t fft_output_buffer[2048];
extern uint16_t chosen_freqs[16];
extern uint8_t uart_freq_buffer[16];
extern int freqs[1024];

// FFT_handler
extern arm_rfft_fast_instance_f32 fft_handler;

#endif /* INC_GLOBAL_VARIABLES_H_ */
