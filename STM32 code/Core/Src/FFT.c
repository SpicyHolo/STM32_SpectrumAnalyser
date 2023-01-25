#include "FFT.h"
#include "global_variables.h"
#include "arm_math.h"
#include "ssd1306.h"

// Returns absolute value of complex number
float abs_complex(float real, float imag)
{
	return sqrtf(real * real + imag * imag);
}

// Displays FFT result on a screen
void DisplayFFT()
{
	ssd1306_Fill(Black);
	switch(display_state)
	{
	case DISPLAY_MANY:
		for(int i =0; i<128; i++) ssd1306_FillRectangle(i, 0, i,(uint8_t)freqs[3*i], White);
		break;
	case DISPLAY_FEW:
		for(int i=0; i<16; i++) ssd1306_FillRectangle(17+6*i, 0, 20+6*i, freqs[chosen_freqs[i]], White);
		break;
	case DISPLAY_COW:
		break;
	default:
		break;
	}
	ssd1306_UpdateScreen();
	buffer_state = FFT_BUFFER_CLEAR;
}

// Calculates FFT
void FFT(UART_HandleTypeDef *huart)
{
	arm_rfft_fast_f32(&fft_handler, fft_input_buffer, fft_output_buffer, 0);

	int freqs_ptr = 0;

	for (int i=0; i<2048; i=i+2)
	{
		freqs[freqs_ptr] = (int)(20*log10f(abs_complex(fft_output_buffer[i], fft_output_buffer[i+1]))) - 150.0f;
		if (freqs[freqs_ptr] < 0)
			freqs[freqs_ptr] = 0;
		++freqs_ptr;
	}

	uart_freq_buffer[0]  = 0xff;
	uart_freq_buffer[1]  = freqs[1];
	uart_freq_buffer[2]  = freqs[2];
	uart_freq_buffer[3]  = freqs[3];
	uart_freq_buffer[4]  = freqs[5];
	uart_freq_buffer[5]  = freqs[7];
	uart_freq_buffer[6]  = freqs[11];
	uart_freq_buffer[7]  = freqs[18];
	uart_freq_buffer[8]  = freqs[29];
	uart_freq_buffer[9]  = freqs[45];
	uart_freq_buffer[10] = freqs[73];
	uart_freq_buffer[11] = freqs[115];
	uart_freq_buffer[12] = freqs[145];
	uart_freq_buffer[13] = freqs[182];
	uart_freq_buffer[14] = freqs[291];
	uart_freq_buffer[15] = freqs[364];

	HAL_UART_Transmit_DMA(huart, uart_freq_buffer, 16);
	buffer_state = FFT_DISPLAY;
}
