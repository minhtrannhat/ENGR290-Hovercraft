#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>


// Constant for Timer1 50 Hz PWM (ICR mode)
#define PWM_TOP 2500

// Converter of [0;255] range of x to [0,PWM_top] range for OCR of Timer1 (16 bit)
#define D1B(x) (uint16_t)(((x)*(uint32_t)(PWM_top))>>8)

#define BAUD 57600UL
#define UBRR ((F_CPU)/((BAUD)*(16UL))-1)

#define SCL_CLOCK 100000L
