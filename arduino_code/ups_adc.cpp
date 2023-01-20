/*
 * Name:		dc_ups.ino
 * Created:	2022-02-11
 * Author:	Isto Saarinen
 */
 

#include "ups_adc.h"





void initADC() {
	// Disable digital buffers
	DIDR0 = (1 << ADC0D) | (1 << ADC1D) | (1 << ADC2D) | (1 << ADC3D) | (1 << ADC4D) | (1 << ADC5D);
	ADMUX = ADC0_BITS;
	ADCSRB = 0;					// not needed
	ADCSRA = (1 << ADEN) |		// Enable ADC
			 (1 << ADSC) |		// Start conversion
			 (1 << ADPS0) |		// Prescaler = 8
			 (1 << ADPS1) |		// Prescaler = ( 128, 125kHz @ 16MHz )
			 (1 << ADPS2);		//             (      125KHz @ 16MHz )
}





uint16_t readADC(uint8_t channel) {
	// Wait untin previous conversion is done
	while ((ADCSRA & (1 << ADSC)) > 0) {
		NOP();
	};
	// Set input channel
	switch (channel) {
	case 0:
		ADMUX = ADC0_BITS;
		break;
	case 1:
		ADMUX = ADC1_BITS;
		break;
	case 2:
		ADMUX = ADC2_BITS;
		break;
	case 3:
		ADMUX = ADC3_BITS;
		break;
	case 4:
		ADMUX = ADC4_BITS;
		break;
	case 5:
		ADMUX = ADC5_BITS;
		break;
	case 6:
		ADMUX = ADC6_BITS;
		break;
	case 7:
		ADMUX = ADC7_BITS;
		break;
	default:
		ADMUX = ADC0_BITS;
		break;
	}
	// Strart conversion
	ADCSRA |= (1 << ADSC);
	// Wait for conversion to finish
	while ((ADCSRA & (1 << ADSC)) > 0) {
		NOP();
	};

	return (ADC);
}

