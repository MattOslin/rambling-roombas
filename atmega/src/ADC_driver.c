/*
 * ADC_driver.c
 *
 * Created: 10/25/2016 6:31:06 PM
 * Author : J. Diego Caporale, Matt Oslin, Garrett Wenger, Jake Welde
 */ 
#include "ADC_driver.h"

/*   WHICH ADCS DO YOU WANT TO USE
*    COMMENT OUT THE ADCS YOU DON'T WANT
*	 DON'T FORGET TO ADJUST #define NUMADCS 
*    NUMADCS SHOULD BE THE NUMBER OF ADCs USED DOESN'T INCLUDE THE ONES WE ADJUST 
*	 DON'T FORGET TO COMMENT OUT THE FINAL SEMICOLON
***********************************************************************************
***********************************************************************************/

#define NUMADCS 8
#define LPF_ALPHA 0.5

const uint8_t ADCsToRead[] = {ADC10, ADC7, ADC5, ADC12, ADC0, ADC1, ADC4, ADC6};

/***********************************************************************************
***********************************************************************************/


void adc_init() {
	//Set voltage reference to Vcc
	set(ADMUX, REFS0);
	clr(ADMUX, REFS1);

	//Set ADC Prescaler to /128 (125kHz ADCClock at 16MHz Sysclock)
	//This will translate to 9kHz read speed for all the ADCS (9kHz/NUMADCS per ADC)
	set(ADCSRA, ADPS0);
	set(ADCSRA, ADPS2);
	set(ADCSRA, ADPS1);

	//Disable Digital Inputs 
	DIDR0 = 0xF3;
	DIDR2 = 0x14;

	//Set ADC Interrupt
	set(ADCSRA, ADIE);

	//Do not use  ADC in FREE RUNNING MODE
	clr(ADCSRA, ADATE);
	
	//Set which ADC to use first (ADC10)
	set(ADCSRB, MUX5); // |= 1 << MUX5; 
	ADMUX = 0x40 + ADC10 % 8;
	// ADMUX = (ADMUX & 0b11111000);// + ADCsToRead[0] % 8;

	// ENABLE ADC AND RUN FIRST READING
	set(ADCSRA, ADEN);
	set(ADCSRA, ADSC);
}

void adc_read(uint16_t rawADCCounts[])
{
	static uint8_t ADCIndex = 0;
	rawADCCounts[ADCIndex] = (1-LPF_ALPHA)*rawADCCounts[ADCIndex] + LPF_ALPHA*ADC;

	//Choose which ADC to run next and set ADMUX register
	ADCIndex = (ADCIndex + 1) % NUMADCS;
  	clr(ADCSRA,ADEN); // turn off ADC while changing registers to avoid spurious behavior

	if (ADCsToRead[ADCIndex] < 8) {
		clr(ADCSRB, MUX5); //ADCSRB &= 0 << MUX5; //clr(ADCSRB, MUX5);
	} else {
		ADCSRB |= 1 << MUX5; //set(ADCSRB, MUX5);
	}

	// ADMUX = (ADMUX & 0b11111000) + ADCsToRead[ADCIndex] % 8;
	ADMUX = 0x40 + ADCsToRead[ADCIndex] % 8;

	set(ADCSRA,ADEN); // re enable ADC

	//Start next ADC read
	set(ADCSRA, ADSC);
}

