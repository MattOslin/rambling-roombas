/*
 * ADC_driver.c
 *
 * Created: 10/25/2016 6:31:06 PM
 * Author : J. Diego Caporale, Matt Oslin, Garrett Wenger, Jake Welde
 */ 

#include "ADC_driver.h"

uint8_t ADCsToRead[NUMADCS];

void adc_init() {
	//Set voltage reference to Vcc
	set(ADMUX, REFS0);
	clr(ADMUX, REFS1);

	//Set ADC Prescaler to /128 (125kHz ADCClock at 16MHz Sysclock)
	//This will translate to 9kHz read speed for all the ADCS (2.4kHz per ADC)
	set(ADCSRA, ADPS0);
	set(ADCSRA, ADPS2);
	set(ADCSRA, ADPS1);


	/*   WHICH ADCS DO YOU WANT TO USE
	*    COMMENT OUT THE ADCS YOU DON'T WANT
	*	 DON'T FORGET TO ADJUST #define NUMADCS 
	***********************************************************************************
	***********************************************************************************/
	
	ADCsToRead = ADC2READ;

	//Disable Digital Inputs 
	disable_ADC_digi();

	/***********************************************************************************
	***********************************************************************************/
	
	//Set ADC Interrupt
	set(ADCSRA, ADIE);

	//Do not use  ADC in FREE RUNNING MODE
	clr(ADCSRA, ADATE);
	
	//Set which ADC to use first
	if (ADCsToRead[0] < 8)
	{
		ADCSRB |= 0 << MUX5; //set(ADCSRB, MUX5);
	}
	else
	{
		ADCSRB |= 1 << MUX5; //clr(ADCSRB, MUX5);
	}

	ADMUX = (ADMUX & 0b11111000) + ADCsToRead[0] % 8;
	// set(ADMUX , MUX2);
	// clr(ADMUX , MUX1);
	// clr(ADMUX , MUX0);

	// ENABLE ADC AND RUN FIRST READING
	set(ADCSRA, ADEN);
	set(ADCSRA, ADSC);
}

void adc_read(int rawADCCounts[])
{
	static uint8_t ADCIndex = 0;
	rawADCCounts[ADCIndex] = ADC;

	//Choose which ADC to run next and set ADMUX register
	ADCIndex = (ADCIndex + 1) % NUMADCS;
	
	if (ADCsToRead[ADCIndex] < 8)
	{
		ADCSRB |= 0 << MUX5; //set(ADCSRB, MUX5);
	}
	else
	{
		ADCSRB |= 1 << MUX5; //clr(ADCSRB, MUX5);
	}
	ADMUX = (ADMUX & 0b11111000) + ADCsToRead[ADCIndex] % 8;

	//Start next ADC read
	set(ADCSRA, ADSC);
}