/*
 * ADC_driver.c
 *
 * Created: 10/25/2016 6:31:06 PM
 * Author : J. Diego Caporale, Matt Oslin, Garrett Wenger, Jake Welde
 */ 
#include "m_general.h"
#include "ADC_driver.h"

/*   WHICH ADCS DO YOU WANT TO USE
*    COMMENT OUT THE ADCS YOU DON'T WANT
*	 DON'T FORGET TO ADJUST #define NUMADCS 
*	 DON'T FORGET TO COMMENT OUT THE FINAL SEMICOLON
***********************************************************************************
***********************************************************************************/

#define NUMADCS 8
const int ADCsToRead[] = {ADC0,ADC1,ADC4,ADC5,ADC6,ADC7/*,ADC8,ADC9*/,ADC10/*,ADC11*/,ADC12,/*ADC13*/};
//Disable Digital Inputs
void adc_dis_digi()
{
set(DIDR0, ADC0D);	//F0
set(DIDR0, ADC1D);	//F1
set(DIDR0, ADC4D);	//F4
set(DIDR0, ADC5D);	//F5
set(DIDR0, ADC6D);	//F6
set(DIDR0, ADC7D);	//F7
// set(DIDR2, ADC8D);	//D4
// set(DIDR2, ADC9D);	//D6
set(DIDR2, ADC10D);	//D7
// set(DIDR2, ADC11D);	//B4 Used in motor controller
set(DIDR2, ADC12D);	//B5
// set(DIDR2, ADC13D);	//B6
}

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

	adc_dis_digi();
	
	//Set ADC Interrupt
	set(ADCSRA, ADIE);

	//Do not use  ADC in FREE RUNNING MODE
	clr(ADCSRA, ADATE);
	
	//Set which ADC to use first
	if (ADCsToRead[0] < 8)
	{
		ADCSRB &= 0 << MUX5; //set(ADCSRB, MUX5);
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

void adc_read(uint16_t rawADCCounts[])
{
	static uint8_t ADCIndex = 0;
	rawADCCounts[ADCIndex] = ADC;
	//toggle(PORTC,6);
	//Choose which ADC to run next and set ADMUX register
	ADCIndex = (ADCIndex + 1) % NUMADCS;
  clr(ADCSRA,ADEN); // turn off ADC while changing registers to avoid spurious behavior

	if (ADCsToRead[ADCIndex] < 8)
	{
		ADCSRB &= 0 << MUX5; //clr(ADCSRB, MUX5);
	}
	else
	{
		ADCSRB |= 1 << MUX5; //set(ADCSRB, MUX5);
	}
	ADMUX = (ADMUX & 0b11111000) + ADCsToRead[ADCIndex] % 8;

  set(ADCSRA,ADEN); // re enable ADC

	//Start next ADC read
	set(ADCSRA, ADSC);
}

