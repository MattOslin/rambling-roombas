/*
 * ADC_driver.h
 *
 * Created: 10/25/2016 6:31:06 PM
 * Author : J. Diego Caporale, Matt Oslin, Garrett Wenger, Jake Welde
 */ 

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

#define NUMADCS 9
#define ADC2READ {ADC0,ADC1,ADC4,ADC5,ADC6,ADC7,ADC8,ADC9,ADC10/*,ADC11,ADC12,ADC13*/}
//Disable Digital Inputs 
#define disable_ADC_digi()	set(DIDR0, ADC0D); \//F0
							set(DIDR0, ADC1D); \//F1
							set(DIDR0, ADC4D); \//F4
							set(DIDR0, ADC5D); \//F5
							set(DIDR0, ADC6D); \//F6
							set(DIDR0, ADC7D); \//F7
							set(DIDR2, ADC8D); \//D4
							set(DIDR2, ADC9D); \//D6
							set(DIDR2, ADC10D)//;\//D7
							// set(DIDR2, ADC11D);\//B4 Used in motor controller
							// set(DIDR2, ADC12D);\//B5
							// set(DIDR2, ADC13D);\//B6

/***********************************************************************************
***********************************************************************************/

void adc_init(void);
void adc_read(int *rawADCCounts);