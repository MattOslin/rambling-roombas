/*
 * ADC_driver.h
 *
 * Created: 10/25/2016 6:31:06 PM
 * Author : J. Diego Caporale, Matt Oslin, Garrett Wenger, Jake Welde
 */ 
#ifndef ADC_DRIVER_H_
#define ADC_DRIVER_H_

#include "m_general.h"

#define ADC0 0
#define ADC1 1
#define ADC4 4
#define ADC5 5
#define ADC6 6
#define ADC7 7
#define ADC8 8
#define ADC9 9
#define ADC10 10
#define ADC11 11
#define ADC12 12
#define ADC13 13

/*   WHICH ADCS DO YOU WANT TO USE
*    COMMENT OUT THE ADCS YOU DON'T WANT
*	 DON'T FORGET TO ADJUST #define NUMADCS 
*	 DON'T FORGET TO COMMENT OUT THE FINAL SEMICOLON
***********************************************************************************
***********************************************************************************/



/***********************************************************************************
***********************************************************************************/

void adc_init();
void adc_read(uint16_t rawADCCounts[]);

#endif /* ADC_DRIVER_H_ */