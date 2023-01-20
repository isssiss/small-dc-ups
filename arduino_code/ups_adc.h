/*
 * Name:		dc_ups.ino
 * Created:	2022-02-11
 * Author:	Isto Saarinen
 */


#ifndef _UPS_ADC_h
#define _UPS_ADC_h

#include <avr/io.h>


// ADC channels
#define ADC_V_IN				0
#define ADC_V_BATT				1
#define ADC_V_OUT				2
#define ADC_A_CGH				3
#define ADC_A_OUT				4


// ADC channel select
#define ADC0_BITS		((1 << REFS0) )												// ADC0 select
#define ADC1_BITS		((1 << REFS0) |                             (1 << MUX0))	// ADC1 select
#define ADC2_BITS		((1 << REFS0) |               (1 << MUX1))					// ADC2 select
#define ADC3_BITS		((1 << REFS0) |               (1 << MUX1) | (1 << MUX0))	// ADC3 select
#define ADC4_BITS		((1 << REFS0) | (1 << MUX2))								// ADC4 select
#define ADC5_BITS		((1 << REFS0) | (1 << MUX2)               | (1 << MUX0))	// ADC5 select
#define ADC6_BITS		((1 << REFS0) | (1 << MUX2) | (1 << MUX1))					// ADC6 select
#define ADC7_BITS		((1 << REFS0) | (1 << MUX2) | (1 << MUX1) | (1 << MUX0))	// ADC7 select

#define NOP() __asm__ __volatile__ ("nop\n\t")



void initADC();

uint16_t readADC(uint8_t channel);



#endif

