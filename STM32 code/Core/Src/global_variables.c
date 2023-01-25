#include "global_variables.h"

uint8_t buffer_state = FFT_BUFFER_CLEAR;
uint8_t display_state = DISPLAY_MANY;
uint32_t adc_buffer[8192] = {0};
float32_t fft_input_buffer[2048] = {0};
float32_t fft_output_buffer[2048] = {0};
uint16_t chosen_freqs[] = {1, 2, 3, 5, 7, 11, 18, 29, 45, 73, 115, 145, 182, 291, 327, 364};
uint8_t uart_freq_buffer[16] = {0};
int freqs[1024] = {0};
arm_rfft_fast_instance_f32 fft_handler;
